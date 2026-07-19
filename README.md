# cat-laser

A Raspberry Pi cat laser toy. A camera watches for a cat; when one shows up,
a servo-gimballed laser dot plays with it using motion patterns designed to
actually read as prey rather than a random dot — darts, freezes, skitters,
retreats, the occasional "let her catch it" moment — then rests, and watches
again.

## How it works

```
main.py            entry point: detection loop + bout/cooldown gating
  |-- CatDetector       Picamera2 + YOLOv8n (ONNX), polls for a cat
  `-- choreographer.py  the actual play logic once a cat is seen
        |-- Runner          fixed-rate playback of a Primitive onto the servos
        |-- Choreographer   picks/sequences primitives into a "bout"
        `-- PCA9685         I2C driver for the servo controller
primitives.py       time-parameterized motion (Dart, Skitter, Circle, ...)
laser_cal.py        interactive servo calibration tool
corner_bounce.py    diagnostic: bounces the dot around the play-box corners
onnx-perf-testing.py  standalone benchmark for detection latency/memory
servo_cal.json      saved calibration + play-box geometry (generated)
```

`main.py` doesn't reimplement any motion logic — it just decides *when*
`Choreographer.bout()` is allowed to run. Detection and servo motion never
happen at the same time: a bout blocks until it's done, then control goes
back to watching for the next cat. This keeps the real-time servo timing
free of YOLO inference jitter.

## Hardware

- Raspberry Pi (tested on Bookworm, 64-bit)
- Pi Camera Module (via Picamera2/libcamera)
- PCA9685 PWM driver board over I2C, at address `0x40`
- 2x standard hobby servos (pan + tilt) on PCA9685 channels 0 and 1
- A laser module driven from GPIO 17 through a transistor/driver — **never
  wire a laser diode directly to a GPIO pin**
- I2C enabled: `raspi-config` → Interface Options → I2C

## Setup

This project uses [`uv`](https://docs.astral.sh/uv/) for dependency
management (`pyproject.toml` / `uv.lock` are both committed).

```bash
uv sync
```

Dependencies include `ultralytics` (YOLO), `adafruit-circuitpython-servokit`,
`gpiozero`, `lgpio`, `rpi-gpio`, `smbus2`, and `evdev`. `picamera2` comes from
the OS image on Raspberry Pi OS rather than PyPI.

## Calibration — do this first

The play box (safe pan/tilt travel limits) and per-servo pulse-width
calibration live in `servo_cal.json`. Before running anything that moves the
servos:

```bash
uv run python laser_cal.py       # interactive min/max/center tuning per servo
uv run python corner_bounce.py   # sanity-check the resulting play-box corners
uv run python choreographer.py --sweep   # walk pan/tilt limits, laser off
```

**Tilt is the eye-safety axis.** Center is horizontal (eye level); the
configured play box only ever points *downward* from there
(`TILT_MIN`/`TILT_MAX` in `primitives.py`). Confirm with `--sweep` that the
dot only ever moves downward on your physical rig before ever pointing it
near a person. Nothing enforces this in software beyond the configured
limits — check it by hand.

## Running it

```bash
uv run python main.py
```

Watches the camera for a cat (YOLOv8n, COCO class "cat", polled every few
seconds). Once one is detected, it runs a bout of choreographed motion,
rests for a cooldown period, then goes back to watching.

Useful flags:

```bash
python main.py --dry                       # no camera, no hardware — prints the sequence
python main.py --once                      # a single bout after the first detection, then exit
python main.py --bout 120 --cooldown 900   # override bout/cooldown length (seconds)
python main.py --poll-interval 2           # how often to check for a cat while idle
python main.py --confidence 0.6            # YOLO confidence threshold
python main.py --laser-brightness 0.5      # laser PWM duty cycle, 0.0-1.0
```

`--dry` swaps in dummy hardware *and* a dummy detector that always reports a
cat present, so the whole bout/cooldown loop can be exercised on a dev
machine with no Pi, camera, or wiring attached.

Ctrl+C at any point finishes the current move, turns the laser off, parks
the servos in a safe downward pose, releases the camera, and exits cleanly.
A second Ctrl+C bails immediately.

## Notes

- `choreographer.py` can still be run standalone (`python choreographer.py`)
  for tuning bout feel without the detection loop in the way.
- Detection is presence-only for now — the choreographer doesn't yet aim
  toward where the cat actually is in frame (`Choreographer.set_cat_hint()`
  exists for this but nothing feeds it real coordinates yet).
- `yolov8n.onnx` is the quantized/exported model used at runtime;
  `yolov8n.pt` is kept around for re-exporting or benchmarking against.