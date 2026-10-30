# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""ChArUco board pose detection and a RealSense colour camera. No arm access.

Board configuration (JSON): type=charuco, dictionary (4x4_50 ...), squares_x, squares_y,
square_length_m, marker_length_m, legacy_pattern, min_corners, max_reprojection_px,
optional image_roi_xyxy, image_exclude_rois_xyxy and image_profile
{color_resolution, fps, crop_xywh}. A measured printed size rescales the board model.
"""
import numpy as np

DICTIONARIES = ("4x4_50", "4x4_100", "5x5_50", "5x5_100", "6x6_50", "6x6_100")


class CharucoDetector:
    def __init__(self, config, measured_size_m=None):
        import cv2
        self.cv, self.cfg = cv2, dict(config)
        if self.cfg.get("type") != "charuco" or self.cfg.get("dictionary") not in DICTIONARIES:
            raise ValueError("board must be type charuco with a supported dictionary")
        sx, sy = self.cfg["squares_x"], self.cfg["squares_y"]
        square, marker = self.cfg["square_length_m"], self.cfg["marker_length_m"]
        if not (type(sx) is int and type(sy) is int and sx >= 3 and sy >= 3 and 0 < marker < square < 1
                and self.cfg.get("min_corners", 6) >= 4 and self.cfg.get("max_reprojection_px", 1.0) > 0):
            raise ValueError("invalid board dimensions or quality thresholds")
        name = "DICT_" + self.cfg["dictionary"].upper()
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        if hasattr(cv2.aruco, "CharucoBoard_create"):  # OpenCV < 4.7
            self.board = cv2.aruco.CharucoBoard_create(sx, sy, square, marker, self.dictionary)
        else:
            self.board = cv2.aruco.CharucoBoard((sx, sy), square, marker, self.dictionary)
        if hasattr(self.board, "setLegacyPattern"):
            self.board.setLegacyPattern(bool(self.cfg.get("legacy_pattern", False)))
        elif not self.cfg.get("legacy_pattern", False) and sy % 2 == 0:
            raise ValueError("even-row modern pattern requires OpenCV >= 4.6; use legacy_pattern")
        points = (self.board.getChessboardCorners() if hasattr(self.board, "getChessboardCorners")
                  else self.board.chessboardCorners)
        self.points = np.asarray(points, dtype=np.float64).copy()
        if measured_size_m is not None:  # Printed board size differs from nominal: scale the model.
            width, height = measured_size_m
            if not (0.5 < width / (sx * square) < 1.5 and 0.5 < height / (sy * square) < 1.5):
                raise ValueError("measured board size is implausible for this board")
            self.points[:, 0] *= width / (sx * square)
            self.points[:, 1] *= height / (sy * square)
            self.cfg["measured_size_m"] = [width, height]
        self.roi = self.cfg.get("image_roi_xyxy")
        self.excluded = self.cfg.get("image_exclude_rois_xyxy", [])
        for box in ([self.roi] if self.roi is not None else []) + list(self.excluded):
            if (not isinstance(box, (list, tuple)) or len(box) != 4 or any(type(v) is not int for v in box)
                    or not (0 <= box[0] < box[2] and 0 <= box[1] < box[3])):
                raise ValueError("image rectangles must be integer [x0, y0, x1, y1]")

    def detect(self, image, K, D):
        """Return (T_camera_board 4x4, quality, annotated image) or raise with the reason."""
        cv = self.cv
        source = image
        if self.roi is not None:  # Keep full-image coordinates; blank everything outside the ROI.
            x0, y0, x1, y1 = self.roi
            if x1 > image.shape[1] or y1 > image.shape[0]:
                raise ValueError("image_roi_xyxy exceeds image bounds")
            source = np.full_like(image, 255)
            source[y0:y1, x0:x1] = image[y0:y1, x0:x1]
        if self.excluded:
            source = source.copy()
            for x0, y0, x1, y1 in self.excluded:
                source[y0:y1, x0:x1] = 255
        corners, ids, _ = cv.aruco.detectMarkers(source, self.dictionary)
        if ids is None:
            raise ValueError("no ChArUco markers detected")
        if len(np.unique(ids)) != len(ids):
            raise ValueError("duplicate marker ids; isolate one board with image_roi_xyxy")
        _, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(
            corners, ids, source, self.board, cameraMatrix=K, distCoeffs=D)
        required = self.cfg.get("min_corners", 6)
        if charuco_ids is None or len(charuco_ids) < required:
            found = 0 if charuco_ids is None else len(charuco_ids)
            raise ValueError(f"too few ChArUco corners ({found}/{required}); check occlusion, blur and framing")
        obj = self.points[charuco_ids.ravel()]
        uv = np.asarray(charuco_corners, dtype=np.float64).reshape(-1, 2)
        if np.linalg.matrix_rank(obj - obj.mean(axis=0), tol=1e-7) < 2:
            raise ValueError("detected corners are collinear")
        ok, rvec, tvec = cv.solvePnP(obj, uv, K, D, flags=cv.SOLVEPNP_ITERATIVE)
        if not ok or not np.isfinite(tvec).all():
            raise ValueError("board pose solve failed")
        rotation = cv.Rodrigues(rvec)[0]
        if np.min((rotation @ obj.T + tvec)[2]) <= 0:
            raise ValueError("board behind camera")
        projected = cv.projectPoints(obj, rvec, tvec, K, D)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((projected - uv) ** 2, axis=1))))
        if not np.isfinite(rms) or rms > self.cfg.get("max_reprojection_px", 1.0):
            raise ValueError(f"reprojection RMS too large: {rms:.3f} px")
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, tvec.ravel()
        annotated = image.copy()
        cv.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids)
        cv.drawFrameAxes(annotated, K, D, rvec, tvec, self.cfg["square_length_m"] * 2)
        quality = dict(corners=int(len(charuco_ids)), reprojection_rms_px=rms, corner_pixels=uv.tolist())
        return transform, quality, annotated


def opencv_distortion(model, coefficients):
    coefficients = np.asarray(coefficients, dtype=float)
    if coefficients.shape != (5,) or not np.isfinite(coefficients).all():
        raise ValueError("invalid camera distortion coefficients")
    if model == "distortion.none":
        return np.zeros(5)
    if model == "distortion.brown_conrady":
        return coefficients
    if model in ("distortion.inverse_brown_conrady", "distortion.modified_brown_conrady") and not coefficients.any():
        return np.zeros(5)
    raise ValueError(f"unsupported nonzero distortion model {model}; rectify first")


def profile_options(profile=None):
    profile = profile or {}
    resolution = profile.get("color_resolution", [640, 480])
    fps = profile.get("fps", 15)
    crop = profile.get("crop_xywh")
    if resolution not in ([640, 480], [1280, 720]) or fps not in (6, 15, 30):
        raise ValueError("unsupported color_resolution or fps")
    if crop is not None:
        if len(crop) != 4 or any(type(v) is not int for v in crop):
            raise ValueError("crop_xywh requires four integers")
        x, y, w, h = crop
        if min(x, y) < 0 or min(w, h) <= 0 or x + w > resolution[0] or y + h > resolution[1]:
            raise ValueError("crop outside the colour image")
    return resolution, fps, crop


class RealSenseCamera:
    """Colour stream only; K is shifted by the crop so poses use full-precision intrinsics."""

    def __init__(self, serial=None, profile=None):
        import pyrealsense2 as rs
        resolution, fps, self.crop = profile_options(profile)
        self.pipeline, self.started = rs.pipeline(), False
        config = rs.config()
        if not serial and len(rs.context().query_devices()) != 1:
            raise ValueError("expected exactly one RealSense camera; pass serial=...")
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.color, *resolution, rs.format.bgr8, fps)
        try:
            started = self.pipeline.start(config)
            self.started = True
            intrinsics = started.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.K = np.array([[intrinsics.fx, 0, intrinsics.ppx], [0, intrinsics.fy, intrinsics.ppy], [0, 0, 1.0]])
            if self.crop is not None:
                self.K[0, 2] -= self.crop[0]
                self.K[1, 2] -= self.crop[1]
            self.D = opencv_distortion(str(intrinsics.model), intrinsics.coeffs)
            self.info = dict(backend="realsense", serial=started.get_device().get_info(rs.camera_info.serial_number),
                             frame="color_optical", width=intrinsics.width, height=intrinsics.height,
                             camera_matrix=self.K.tolist(), dist_coeffs=self.D.tolist(),
                             distortion_model=str(intrinsics.model))
            if self.crop is not None:
                self.info.update(width=self.crop[2], height=self.crop[3], source_resolution=resolution,
                                 crop_xywh=list(self.crop), fps=fps)
            for _ in range(15):  # Auto exposure settles on the first frames.
                self.capture()
        except BaseException:
            self.close()
            raise

    def capture(self):
        frame = self.pipeline.wait_for_frames(3000).get_color_frame()
        if not frame:
            raise RuntimeError("missing colour frame")
        image = np.asanyarray(frame.get_data())
        if self.crop is None:
            return image.copy()
        x, y, w, h = self.crop
        return image[y:y + h, x:x + w].copy()

    def close(self):
        if self.started:
            self.pipeline.stop()
            self.started = False
