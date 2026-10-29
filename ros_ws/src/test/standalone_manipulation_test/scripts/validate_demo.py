#!/usr/bin/env python3

import subprocess
import sys

EXPECTED_STATES = (
    "WAITING_FOR_SIMULATION",
    "INITIAL_RELEASE",
    "SAFE_START",
    "PREGRASP",
    "APPROACH_PICK",
    "GRASP",
    "LIFT",
    "TRANSFER",
    "PLACE",
    "RELEASE",
    "RETREAT",
    "SUCCEEDED",
)


def main():
    try:
        result = subprocess.run(
            [
                "ros2",
                "launch",
                "stoneware_manipulation_test",
                "demo.launch.py",
                "headless:=true",
            ],
            capture_output=True,
            text=True,
            timeout=180,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        print(output, end="")
        print("FAIL: pick-and-place launch exceeded 180 seconds", file=sys.stderr)
        return 1

    output = result.stdout + result.stderr
    print(output, end="")
    if result.returncode != 0:
        print(f"FAIL: launch exited with status {result.returncode}", file=sys.stderr)
        return 1

    markers = [f"DEMO_STATE {state}" for state in EXPECTED_STATES]
    positions = [output.find(marker) for marker in markers]
    missing = [marker for marker, position in zip(markers, positions) if position < 0]
    if missing:
        print(f"FAIL: missing expected state(s): {', '.join(missing)}", file=sys.stderr)
        return 1
    if positions != sorted(positions):
        print("FAIL: manipulation states were not reached in the expected order", file=sys.stderr)
        return 1

    print("PASS: robot, controllers, grasp, transfer, release, and safe return verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
