#!/usr/bin/env python3
"""
main.py — the actual toy: watch for a cat, then play.

Ties the pieces together into one loop:

    watch camera -> cat detected? -> run a bout (choreographer.py)
                                   -> cooldown -> watch again

Detection and motion never run at the same time. YOLO inference and
real-time servo timing both want the CPU; interleaving them on a Pi would
make the dart/freeze timing jittery. So this polls for a cat at a fairly
slow cadence while idle, and once a bout starts it hands full control to
Choreographer.bout(), which blocks until the bout ends.

Run:
    python main.py                  # normal operation, go until Ctrl+C
    python main.py --dry            # no hardware, no camera — prints the sequence
    python main.py --once           # one bout (after one detection), then exit
    python main.py --poll-interval 2 --confidence 0.6
    python main.py --bout 120 --cooldown 900

Ctrl+C at any point: laser off, servos parked, camera released, clean exit.
"""

import argparse
import random
import signal
import sys
import time

from choreographer import (
    BOUT_SECONDS,
    COOLDOWN_SECONDS,
    LASER_BRIGHTNESS,
    LASER_GPIO_PIN,
    PARK_TILT,
    Choreographer,
    DimmableLaser,
    DummyLaser,
    DummyPCA,
    PCA9685,
    Runner,
)
from primitives import Aim, load_calibration

# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------
CAT_CLASS = 15          # COCO class id for "cat"
MODEL_PATH = "yolov8n.onnx"
POLL_INTERVAL = 0.5     # seconds between frames while idle/watching
CONFIDENCE = 0.2
FRAME_SIZE = (640, 640)


class CatDetector:
    """One YOLO pass per check(); everything else stays idle in between."""

    def __init__(self, model_path=MODEL_PATH, conf=CONFIDENCE, frame_size=FRAME_SIZE):
        from picamera2 import Picamera2
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.conf = conf
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": frame_size, "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)  # let auto-exposure/white-balance settle

    def check(self):
        """Grab one frame and return True if a cat is in it."""
        frame = self.picam2.capture_array()  # RGB888 numpy array
        results = self.model(frame, classes=[CAT_CLASS], conf=self.conf, verbose=False)
        return len(results[0].boxes) > 0

    def close(self):
        self.picam2.stop()


class DummyDetector:
    """Stands in for CatDetector under --dry, or off-Pi for a sanity check.
    Always reports a cat present so the bout/cooldown loop can be exercised
    without a camera attached."""

    def check(self):
        return True

    def close(self):
        pass


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--dry", action="store_true",
                    help="no hardware, no camera; DummyDetector always 'sees' a cat")
    ap.add_argument("--once", action="store_true",
                    help="run a single bout after the first detection, then exit")
    ap.add_argument("--bout", type=float, default=BOUT_SECONDS,
                    help="bout length in seconds")
    ap.add_argument("--cooldown", type=float, default=COOLDOWN_SECONDS,
                    help="rest between bouts in seconds")
    ap.add_argument("--poll-interval", type=float, default=POLL_INTERVAL,
                    help="seconds between detection checks while watching")
    ap.add_argument("--confidence", type=float, default=CONFIDENCE,
                    help="YOLO confidence threshold for a cat detection")
    ap.add_argument("--model", default=MODEL_PATH,
                    help="path to the YOLO model (.onnx or .pt)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--laser-brightness", type=float, default=LASER_BRIGHTNESS,
                    help="laser PWM duty cycle from 0.0 to 1.0")
    args = ap.parse_args()
    if not 0.0 <= args.laser_brightness <= 1.0:
        ap.error("--laser-brightness must be between 0.0 and 1.0")

    cal = load_calibration()

    if args.dry:
        pca, laser, detector = DummyPCA(), DummyLaser(), DummyDetector()
    else:
        pca = PCA9685()
        laser = DimmableLaser(LASER_GPIO_PIN, brightness=args.laser_brightness)
        laser.off()
        detector = CatDetector(model_path=args.model, conf=args.confidence)

    runner = Runner(pca, cal, dry=args.dry)
    chor = Choreographer(runner, rng=random.Random(args.seed))

    # Ctrl+C sets a flag rather than raising mid-primitive, so the current
    # move (or detection poll) finishes cleanly and we exit through the
    # parking path. Same pattern as choreographer.py.
    def on_sigint(_sig, _frm):
        if runner.stop:
            sys.exit(1)  # second Ctrl+C = bail now
        print("\nStopping after this move...")
        runner.stop = True

    signal.signal(signal.SIGINT, on_sigint)

    # First physical write: forward + safely downward. Never park at tilt=0
    # (horizontal / eye level).
    runner.write(Aim(0.0, PARK_TILT))
    time.sleep(0.3)

    try:
        n = 0
        while not runner.stop:
            print("\nWatching for a cat...")
            while not runner.stop and not detector.check():
                time.sleep(args.poll_interval)
            if runner.stop:
                break

            n += 1
            print(f"Cat detected! === bout {n} ({args.bout:.0f}s) ===")
            chor.bout(seconds=args.bout, laser=laser)

            if args.once or runner.stop:
                break

            print(f"--- cooldown {args.cooldown / 60:.1f} min (Ctrl+C to quit) ---")
            end = time.monotonic() + args.cooldown
            while time.monotonic() < end and not runner.stop:
                time.sleep(0.2)
    finally:
        laser.off()
        detector.close()
        if not args.dry:
            try:
                runner.stop = False  # allow the park glide to run
                runner.glide_to(Aim(0.0, PARK_TILT), duration=0.7)
            except Exception:
                pass
            laser.close()
        print("Laser off, parked, camera released.")


if __name__ == "__main__":
    main()