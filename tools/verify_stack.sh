#!/usr/bin/env bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Complete software verification only: no CAN, camera, enable, disable or real arm motion.
set -euo pipefail
SDK_ROOT="$(cd "$(dirname "$(realpath -- "${BASH_SOURCE[0]}")")/../../../.." && pwd -P)"
cd "${SDK_ROOT}"
STACK_BUILD="$(realpath -m -- "${1:-${SDK_ROOT}/output/manipulation-dev}")"
# Every run installs into an empty prefix: files left by an earlier run must not change what the
# installation checks see.
[[ "${STACK_BUILD}" != "/" ]] || { echo "verify_stack: the build directory must not be /" >&2; exit 2; }
rm -rf -- "${STACK_BUILD}/install"
# Tests use a private lease directory so they never contend with programs on this host.
export MANIPULATION_LOCK_DIRECTORY="${STACK_BUILD}/locks"
export PYTHONDONTWRITEBYTECODE=1
python3 components/control/manipulation/tools/check_environment.py --profile build
cmake -S components/control/manipulation -B "${STACK_BUILD}/core" \
    -DCMAKE_BUILD_TYPE=Release -DMANIPULATION_ENABLE_NERO=ON \
    -DMANIPULATION_ENABLE_CALIBRATION=ON -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_INSTALL_PREFIX="${STACK_BUILD}/install"
cmake --build "${STACK_BUILD}/core" -j2
ctest --test-dir "${STACK_BUILD}/core" --output-on-failure
cmake --install "${STACK_BUILD}/core"
cmake -S application/native/dice_arena -B "${STACK_BUILD}/arena" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_INSTALL_PREFIX="${STACK_BUILD}/install" \
    -DMANIPULATION_DEVICE="${STACK_BUILD}/core/manipulation-device"
cmake --build "${STACK_BUILD}/arena" -j2
ctest --test-dir "${STACK_BUILD}/arena" --output-on-failure
cmake --install "${STACK_BUILD}/arena"
cmake -S components/thirdparty/pyagxarm -B "${STACK_BUILD}/vendor" \
    -DCMAKE_INSTALL_PREFIX="${STACK_BUILD}/install"
cmake --install "${STACK_BUILD}/vendor"
python3 application/native/dice_arena/tests/check_runtime.py "${STACK_BUILD}/install/share/dice_arena/game"
cmake -S components/control/manipulation/examples/device_consumer -B "${STACK_BUILD}/consumer" \
    -DCMAKE_PREFIX_PATH="${STACK_BUILD}/install"
cmake --build "${STACK_BUILD}/consumer" -j2
"${STACK_BUILD}/consumer/device-state" components/control/manipulation/examples/mock_device.yaml
cmake -S components/control/manipulation -B "${STACK_BUILD}/default" -DCMAKE_BUILD_TYPE=Release
cmake --build "${STACK_BUILD}/default" -j2
ctest --test-dir "${STACK_BUILD}/default" --output-on-failure
cmake -S components/control/manipulation -B "${STACK_BUILD}/extension" -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DMANIPULATION_EXTRA_DRIVER_DIRS="${SDK_ROOT}/components/control/manipulation/examples/bench_driver"
cmake --build "${STACK_BUILD}/extension" -j2
"${STACK_BUILD}/extension/manipulation-device" run \
    components/control/manipulation/examples/bench_driver/bench.yaml
"${STACK_BUILD}/install/bin/manipulation-device" run \
    components/control/manipulation/examples/point_sequence/point_sequence.yaml
python3 components/control/manipulation/tests/check_relocation.py "${STACK_BUILD}/install"
python3 components/control/manipulation/tests/check_commands.py "${STACK_BUILD}/install"
