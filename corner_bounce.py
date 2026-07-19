#!/usr/bin/env python3
"""Bounce the laser around the four corners of the configured safe play box.

Uses the existing servo calibration and the PAN/TILT play-box limits from
primitives.py. Ctrl+C turns the laser off and parks the servos.

Examples:
    python corner_bounce.py
    python corner_bounce.py --seconds 30
    python corner_bounce.py --move-time 1.2 --pause 0.15
    python corner_bounce.py --inset 0.03
"""

import argparse
import signal
import time

from gpiozero import LED

from choreographer import LASER_GPIO_PIN, PCA9685, Runner
from primitives import (
    Aim,
    Dart,
    PAN_MAX,
    PAN_MIN,
    TILT_MAX,
    TILT_MIN,
    load_calibration,
    min_dart_duration,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 runs until Ctrl+C",
    )
    parser.add_argument(
        "--move-time",
        type=float,
        default=0.9,
        help="requested seconds per corner-to-corner move",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=0.12,
        help="pause at each corner in seconds",
    )
    parser.add_argument(
        "--inset",
        type=float,
        default=0.0,
        help="move this far inward from every play-box edge, e.g. 0.03",
    )
    args = parser.parse_args()

    inset = max(0.0, args.inset)
    if PAN_MIN + inset >= PAN_MAX - inset or TILT_MIN + inset >= TILT_MAX - inset:
        parser.error("--inset is too large for the configured play box")

    # Clockwise corners of the safe play box. These are normalized play-box
    # limits, not the raw min/max pulse values from servo_cal.json.
    corners = [
        Aim(PAN_MIN + inset, TILT_MIN + inset),
        Aim(PAN_MAX - inset, TILT_MIN + inset),
        Aim(PAN_MAX - inset, TILT_MAX - inset),
        Aim(PAN_MIN + inset, TILT_MAX - inset),
    ]

    print(corners)

    cal = load_calibration()
    pca = PCA9685()
    laser = LED(LASER_GPIO_PIN)
    runner = Runner(pca, cal)

    def stop(_sig=None, _frame=None) -> None:
        runner.stop = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    start_time = time.monotonic()
    laser.off()

    try:
        # Move into the first corner with the laser off, then begin tracing.
        first = corners[0]
        duration = max(
            args.move_time,
            min_dart_duration(runner.current, first, cal),
        )
        runner.play(Dart(runner.current, first, duration=duration))
        laser.on()

        index = 1
        while not runner.stop:
            if args.seconds > 0 and time.monotonic() - start_time >= args.seconds:
                break

            target = corners[index]
            duration = max(
                args.move_time,
                min_dart_duration(runner.current, target, cal),
            )
            runner.play(Dart(runner.current, target, duration=duration))

            if runner.stop:
                break
            if args.pause > 0:
                time.sleep(args.pause)

            index = (index + 1) % len(corners)

    finally:
        laser.off()
        try:
            runner.stop = False
            park = Aim(0.0, 0.2)
            duration = max(0.8, min_dart_duration(runner.current, park, cal))
            runner.play(Dart(runner.current, park, duration=duration))
        except Exception:
            pass
        laser.close()
        print("Laser off, parked.")


if __name__ == "__main__":
    main()