#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Download pinned simulation meshes without modifying bundled model definitions."""

import argparse
import hashlib
import json
import os
import re
import tempfile
import urllib.parse
import urllib.request
from pathlib import Path, PurePosixPath


DEFAULT_MANIFEST = (
    Path(__file__).resolve().parents[1] / "descriptions/simulation/ur5e_robotiq/assets.json"
)
BLOCK_SIZE = 1024 * 1024


def sha256_file(path):
    """Return the SHA-256 digest of a file without loading it into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(BLOCK_SIZE), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_entry(relative_path, digest):
    """Reject unsafe paths and malformed hashes before any filesystem changes."""
    if not isinstance(relative_path, str) or not relative_path:
        raise ValueError("asset path must be a nonempty relative POSIX path")
    path = PurePosixPath(relative_path)
    if (
        not path.parts
        or path.is_absolute()
        or path.as_posix() != relative_path
        or any(part in (".", "..") for part in path.parts)
        or any(character in relative_path for character in "\\:%?#\x00")
    ):
        raise ValueError(f"unsafe asset path: {relative_path}")
    if not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None:
        raise ValueError(f"invalid SHA-256 digest: {relative_path}")


def load_manifest(path):
    """Load a versioned manifest containing source_url and file/hash pairs."""
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or set(manifest) != {
        "schema_version",
        "source_url",
        "files",
    }:
        raise ValueError("invalid simulation asset manifest")
    if type(manifest["schema_version"]) is not int or manifest["schema_version"] != 1:
        raise ValueError("asset manifest requires schema_version: 1")
    source = manifest["source_url"]
    if not isinstance(source, str) or urllib.parse.urlparse(source).scheme != "https":
        raise ValueError("manifest source_url must use HTTPS")
    files = manifest["files"]
    if not isinstance(files, dict) or not files:
        raise ValueError("asset manifest requires a nonempty files mapping")
    for relative_path, digest in files.items():
        validate_entry(relative_path, digest)
    return manifest


def prepare_assets(manifest, output_root, check_only=False, base_url=None, opener=None):
    """Verify or fetch meshes atomically, preserving existing files on failures.

    Args:
        manifest: Validated manifest returned by load_manifest.
        output_root: Directory for the paths listed in manifest.files.
        check_only: Verify without creating directories or accessing the network.
        base_url: Optional mirror URL; hashes always come from the manifest.
        opener: Optional urllib-compatible opener for isolated transport tests.

    Returns:
        Number of files downloaded. Raises on missing files or digest mismatch.
    """
    root = Path(output_root).resolve()
    source = (base_url or manifest["source_url"]).rstrip("/")
    parsed_source = urllib.parse.urlparse(source)
    if parsed_source.scheme not in ("https", "http", "file"):
        raise ValueError("asset source must use HTTPS, HTTP or a local file URL")
    if parsed_source.query or parsed_source.fragment:
        raise ValueError("asset source cannot contain a query or fragment")
    open_url = opener or urllib.request.urlopen
    destinations = []
    for relative_path, digest in manifest["files"].items():
        validate_entry(relative_path, digest)
        destination = root / relative_path
        if not destination.resolve().is_relative_to(root):
            raise ValueError(f"asset path escapes output directory: {relative_path}")
        destinations.append((relative_path, digest, destination))

    missing = []
    downloaded = 0
    for relative_path, digest, destination in destinations:
        if destination.is_file() and sha256_file(destination) == digest:
            continue
        if check_only:
            missing.append(relative_path)
            continue
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                dir=destination.parent, prefix=f".{destination.name}.", suffix=".part", delete=False
            ) as stream:
                temporary = Path(stream.name)
                hasher = hashlib.sha256()
                url = f"{source}/{urllib.parse.quote(relative_path, safe='/')}"
                with open_url(url, timeout=60) as response:
                    for block in iter(lambda: response.read(BLOCK_SIZE), b""):
                        stream.write(block)
                        hasher.update(block)
                stream.flush()
                os.fsync(stream.fileno())
            if hasher.hexdigest() != digest:
                raise RuntimeError(f"SHA-256 mismatch: {relative_path}")
            os.replace(temporary, destination)
            downloaded += 1
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    if missing:
        raise RuntimeError("missing or invalid simulation assets: " + ", ".join(missing))
    return downloaded


def main(argv=None):
    """Prepare or verify a model's meshes using a source/install-relative manifest."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path, help="mesh root (default: MANIFEST_DIR/models)")
    parser.add_argument("--base-url", help="asset mirror URL; pinned hashes remain mandatory")
    parser.add_argument("--check", action="store_true", help="verify files without downloading")
    arguments = parser.parse_args(argv)
    try:
        manifest = load_manifest(arguments.manifest)
        output = arguments.output or arguments.manifest.resolve().parent / "models"
        downloaded = prepare_assets(manifest, output, arguments.check, arguments.base_url)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f"simulation assets: {error}\n")
    print(f"simulation assets: {len(manifest['files'])} files ready, {downloaded} downloaded")


if __name__ == "__main__":
    main()
