#!/usr/bin/env bash
# Builds the real firmware sources against the mock Arduino core in test/mock/ and runs the simulator tests.
#   ./test/run_tests.sh                 run everything
#   ./test/run_tests.sh arena markers   run only the named tests (see TESTS[] in test_main.cpp)
#   CXX="g++" ./test/run_tests.sh       choose the compiler yourself
#   EXTRA_FLAGS="-DFINISH_STOP_ENABLED=true" ./test/run_tests.sh arena     build a configuration variant
# Needs any C++17 compiler: g++, clang++ or Zig ("pip install ziglang" then CXX="python -m ziglang c++").
set -euo pipefail
cd "$(dirname "$0")/.."

if [ -z "${CXX:-}" ]; then
  if command -v g++ >/dev/null 2>&1; then CXX="g++"
  elif command -v clang++ >/dev/null 2>&1; then CXX="clang++"
  elif python -m ziglang version >/dev/null 2>&1; then CXX="python -m ziglang c++"
  else echo "No C++ compiler found. Install g++/clang++, or: pip install ziglang" >&2; exit 2
  fi
fi

OUT="${OUT_DIR:-test/build}"
mkdir -p "$OUT"
EXE="$OUT/line_follower_tests"
echo "compiler: $CXX"
$CXX -std=c++17 -O1 -Wall -Wextra -Wno-nullability-completeness ${EXTRA_FLAGS:-} -I test/mock -I line_follower_robot \
  test/test_main.cpp test/mock/mock_arduino.cpp \
  line_follower_robot/sensors.cpp line_follower_robot/motor.cpp line_follower_robot/navigation.cpp \
  line_follower_robot/diagnostics.cpp line_follower_robot/robot.cpp \
  -o "$EXE"
"$EXE" "$@"
