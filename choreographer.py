#!/usr/bin/env python3
"""
choreographer.py — runs the laser toy until Ctrl+C.

Structure:
    session -> bout (a few minutes of play) -> cooldown -> bout -> ...
    bout    -> move, freeze, move, freeze, ...

The freezes are not filler. Cats commit to a pounce on prey that has STOPPED.
A dot in constant motion gets abandoned in under a minute.

Intensity decays over each bout: fast darts and short freezes early, slower
and longer later, like a real critter tiring. Occasionally the dot "gets
caught" — slows to something catchable, holds, then startles away. Cats need
wins or they disengage.

Run:
    python choreographer.py            # go until Ctrl+C
    python choreographer.py --dry      # no hardware, prints the sequence
    python choreographer.py --once     # a single bout, then exit
    python choreographer.py --bout 120 --cooldown 900
    python choreographer.py --sweep    # walk the play box edges, then exit

Ctrl+C at any point: laser off, servos parked, clean exit.
"""

import argparse
import math
import random
import signal
import sys
import time

from primitives import (
    DT, PAN_MAX, PAN_MIN, SERVO_DOWN_CH, SERVO_MAX_DEG_PER_S, SERVO_UP_CH,
    TILT_MAX, TILT_MIN, UPDATE_HZ,
    Aim, BackAndForth, Circle, Dart, Freeze, Skitter,
    clamp_to_box, load_calibration, min_dart_duration, norm_to_deg,
    norm_to_pulse, peak_speed, random_point, retreat,
)

# ---------------------------------------------------------------------------
# Session shape
# ---------------------------------------------------------------------------
BOUT_SECONDS = 180.0        # hard cap on one play session
COOLDOWN_SECONDS = 1200.0   # rest between bouts. Don't let it run all day.
LASER_GPIO_PIN = 17

INTENSITY_START = 0.90
INTENSITY_END = 0.30
LASER_BRIGHTNESS = 1  # 0.0=off, 1.0=full power
P_CATCH = 0.12              # chance a move turns into a "let her win" beat

# Rest / startup pose: facing forward (pan center), tilted safely DOWNWARD.
# Derived from the play box so it always sits inside the eye-safe tilt range
# and can never accidentally park at horizontal (tilt=0) again.
PARK_TILT = (TILT_MIN + TILT_MAX) / 2.0

# ---------------------------------------------------------------------------
# PCA9685 (block-write variant: 1 I2C transaction per channel instead of 4)
# ---------------------------------------------------------------------------
I2C_BUS = 1
I2C_ADDR = 0x40
PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x06
PWM_FREQ = 60
# PCA9685 MODE1 bits
MODE1_RESTART = 0x80
MODE1_AI = 0x20       # auto-increment registers during block writes
MODE1_SLEEP = 0x10

# Compatibility with the existing servo_cal.py calibration. The saved
# min_ms/max_ms values were tuned while this prescale correction was active.
# Removing it changes the physical pulse width for the same saved value and
# can drive a servo beyond its calibrated mechanical limits.
PCA9685_OSC_HZ = 25_000_000.0
PRESCALE_FUDGE = 0.8449


class PCA9685:
    def __init__(self, bus=I2C_BUS, addr=I2C_ADDR):
        import smbus2
        try:
            self.bus = smbus2.SMBus(bus)
        except FileNotFoundError as e:
            raise RuntimeError(
                f"I2C bus {bus} unavailable. raspi-config -> Interface -> I2C."
            ) from e
        self.addr = addr
        try:
            # Probe the device, then explicitly enable register auto-increment.
            # set_pwm() performs a 4-byte block write and requires MODE1.AI.
            self._w(PCA9685_MODE1, MODE1_AI)
        except OSError as e:
            raise RuntimeError(
                f"No PCA9685 at {hex(addr)} on bus {bus}. Check `i2cdetect -y 1`."
            ) from e
        time.sleep(0.01)
        self.set_pwm_freq(PWM_FREQ)

    def _w(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _r(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def set_pwm_freq(self, freq_hz):
        ideal = PCA9685_OSC_HZ / (4096.0 * float(freq_hz)) - 1.0
        prescale = int(round(ideal * PRESCALE_FUDGE))
        prescale = max(3, min(255, prescale))

        old = self._r(PCA9685_MODE1)
        awake = (old | MODE1_AI) & ~MODE1_SLEEP
        self._w(PCA9685_MODE1, awake | MODE1_SLEEP)
        self._w(PCA9685_PRESCALE, prescale)
        self._w(PCA9685_MODE1, awake)
        time.sleep(0.005)
        self._w(PCA9685_MODE1, awake | MODE1_RESTART)
        time.sleep(0.005)

    def set_pwm(self, channel, on, off):
        # MODE1.AI is enabled in __init__, so these bytes land in
        # ON_L, ON_H, OFF_L, and OFF_H respectively.
        self.bus.write_i2c_block_data(
            self.addr, LED0_ON_L + 4 * channel,
            [on & 0xFF, (on >> 8) & 0xFF, off & 0xFF, (off >> 8) & 0xFF])

    def set_pulse_ms(self, channel, pulse_ms):
        # Keep the same nominal conversion used when servo_cal.json was created.
        # The prescale fudge above and this conversion must remain paired.
        off = round(pulse_ms * PWM_FREQ * 4096.0 / 1000.0)
        self.set_pwm(channel, 0, max(0, min(4095, off)))


class DummyPCA:
    def set_pulse_ms(self, channel, pulse_ms):
        pass


class DummyLaser:
    def on(self):
        pass

    def off(self):
        pass

    def close(self):
        pass


class DimmableLaser:
    """PWM-controlled laser output with a capped on-state brightness.

    This assumes GPIO 17 drives the laser through a suitable transistor or
    laser-driver enable input. Do not power a laser module directly from a
    Raspberry Pi GPIO pin.
    """

    def __init__(self, pin, brightness=LASER_BRIGHTNESS, frequency=1000):
        from gpiozero import PWMLED
        self.brightness = max(0.0, min(1.0, float(brightness)))
        self.device = PWMLED(pin, frequency=frequency, initial_value=0.0)

    def on(self):
        self.device.value = self.brightness

    def off(self):
        self.device.value = 0.0

    def close(self):
        self.device.close()


# ---------------------------------------------------------------------------
# Runner — samples a primitive at a fixed rate and pushes it out
# ---------------------------------------------------------------------------
class Runner:
    def __init__(self, pca, cal, dry=False):
        self.pca = pca
        self.cal = cal
        self.dry = dry
        self.pan_cal = cal[str(SERVO_DOWN_CH)]
        self.tilt_cal = cal[str(SERVO_UP_CH)]
        # Start the internal pose forward + safely downward, matching the
        # first physical write in main(), so the first primitive glides from
        # a real, eye-safe pose rather than from horizontal.
        self.current = Aim(0.0, PARK_TILT)
        self.stop = False

    def write(self, aim):
        self.current = aim
        self.pca.set_pulse_ms(SERVO_DOWN_CH, norm_to_pulse(aim.pan, self.pan_cal))
        self.pca.set_pulse_ms(SERVO_UP_CH, norm_to_pulse(aim.tilt, self.tilt_cal))

    def play(self, prim):
        """Fixed-rate, drift-free. The deadline accumulator matters: naive
        sleep(DT) adds the body's runtime every tick and the motion goes flat."""
        if self.dry:
            pp, pt = peak_speed(prim, self.cal)
            flag = "  <-- SATURATES" if max(pp, pt) > SERVO_MAX_DEG_PER_S else ""
            print(f"  {prim.__class__.__name__:10s} {prim.duration:5.2f}s  "
                  f"peak {max(pp, pt):6.1f} d/s{flag}")
            self.current = prim.end()
            return

        t = 0.0
        next_t = time.monotonic()
        while t < prim.duration and not self.stop:
            self.write(prim.at(t))
            next_t += DT
            t += DT
            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.monotonic()  # behind — resync, don't spiral
        if not self.stop:
            self.write(prim.end())

    def glide_to(self, target, duration=0.8):
        self.play(Dart(self.current, target, duration=duration))


# ---------------------------------------------------------------------------
# Choreographer
# ---------------------------------------------------------------------------
class Choreographer:
    """Samples primitives into something that reads as an animal.

    cat_hint is where the cat is, in play-box coords. It stays None unless
    something sets it — the detector can call set_cat_hint() later. Retreat
    is simply not sampled while it's None, so this runs standalone today and
    gets better when the detector lands.
    """

    def __init__(self, runner, rng=None, verbose=True):
        self.r = runner
        self.cal = runner.cal
        self.rng = rng or random.Random()
        self.verbose = verbose
        self.cat_hint = None
        self.last_kind = None

    def set_cat_hint(self, aim):
        self.cat_hint = clamp_to_box(*aim) if aim else None

    # -- move selection ----------------------------------------------------
    def _weights(self):
        w = {"dart": 0.24, "back_forth": 0.31, "skitter": 0.16, "circle": 0.16, "long_dart": 0.21}
        if self.cat_hint is not None:
            w["retreat"] = 0.28
        # don't do the same thing twice in a row — that's what reads as robotic
        if self.last_kind in w:
            w[self.last_kind] *= 0.25
        return w

    def _pick(self):
        w = self._weights()
        kinds, vals = list(w.keys()), list(w.values())
        total = sum(vals)
        x = self.rng.uniform(0, total)
        acc = 0.0
        for k, v in zip(kinds, vals):
            acc += v
            if x <= acc:
                return k
        return kinds[-1]


    def _far_point(self, here, min_pan_distance=0.65, min_total_distance=0.72):
        """Pick a destination far from the current point without changing the
        calibrated play-box limits. Prefer the opposite pan side so the dot
        makes a visible room-crossing chase.
        """
        rng = self.rng
        candidates = []
        for _ in range(24):
            p = random_point(rng, margin=0.02)
            pan_d = abs(p.pan - here.pan)
            total_d = math.hypot(p.pan - here.pan, p.tilt - here.tilt)
            candidates.append((pan_d + 0.35 * total_d, p))
            if pan_d >= min_pan_distance and total_d >= min_total_distance:
                return p

        # The box can be asymmetric around the current point. Fall back to
        # the best sampled destination rather than ever exceeding the limits.
        return max(candidates, key=lambda item: item[0])[1]

    def _dart(self, here, target, duration, bow=0.0):
        """Never ask for a dart the servo can't actually track. Requested
        speed is a wish; min_dart_duration is the physics."""
        floor = min_dart_duration(here, target, self.cal)
        return Dart(here, target, duration=max(duration, floor), bow=bow)

    def _clamp(self, dart):
        """Same floor, for Darts built elsewhere (retreat() makes its own)."""
        dart.duration = max(dart.duration,
                            min_dart_duration(dart.start, dart.target, self.cal))
        return dart

    def _make_move(self, kind, here, intensity):
        rng = self.rng
        fast = lambda lo, hi: lerp_(lo, hi, intensity)

        if kind == "dart":
            # Most ordinary darts should still cover meaningful ground.
            target = self._far_point(here, min_pan_distance=0.35,
                                     min_total_distance=0.42)
            return self._dart(here, target,
                              duration=fast(0.70, 0.30),
                              bow=rng.uniform(-0.10, 0.10))

        if kind == "long_dart":
            # A true room-crossing "runaway" move. The destination is still
            # clamped to the existing calibrated play box.
            far = self._far_point(here)
            return self._dart(here, far, duration=fast(1.05, 0.48),
                              bow=rng.uniform(-0.14, 0.14))

        if kind == "back_forth":
            far = self._far_point(here, min_pan_distance=0.50,
                                  min_total_distance=0.56)
            # Calculate a safe per-leg duration using the same servo speed
            # model as Dart. BackAndForth uses smoothstep, whose peak slope is
            # lower than Dart's cubic ease, so the Dart floor is conservative.
            leg_floor = min_dart_duration(here, far, self.cal)
            leg_time = max(fast(0.68, 0.36), leg_floor)
            return BackAndForth(here, far,
                                legs=rng.choice((3, 4, 5, 6)),
                                leg_duration=leg_time,
                                bow=rng.uniform(-0.055, 0.055))

        if kind == "skitter":
            near = clamp_to_box(here.pan + rng.uniform(-0.34, 0.34),
                                here.tilt + rng.uniform(-0.19, 0.19))
            return Skitter(here, near, duration=fast(1.9, 0.9),
                           amplitude=rng.uniform(0.03, 0.07), rng=rng)

        if kind == "circle":
            # Choose the circle from the space actually available around its
            # center. This produces a mix of medium and genuinely large loops
            # without widening the calibrated play box or clipping the edges.
            # ASPECT is already applied inside Circle, so vertical clearance
            # is converted back into an equivalent horizontal radius.
            from primitives import ASPECT, PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX

            # Keep the center away from the extreme edges so large circles are
            # possible more often. The current point is still blended smoothly
            # onto the selected orbit by Circle itself.
            center = Aim(
                rng.uniform(PAN_MIN + 0.10, PAN_MAX - 0.10),
                rng.uniform(TILT_MIN + 0.06, TILT_MAX - 0.06),
            )
            pan_room = min(center.pan - PAN_MIN, PAN_MAX - center.pan)
            tilt_room = min(center.tilt - TILT_MIN, TILT_MAX - center.tilt)
            max_radius = max(0.10, min(pan_room, tilt_room / max(ASPECT, 1e-6)))

            # Weighted size bands: circles can still be compact, but most are
            # now medium/large and scale to the available safe space.
            size_roll = rng.random()
            if size_roll < 0.15:
                radius = rng.uniform(0.28, 0.42) * max_radius
            elif size_roll < 0.55:
                radius = rng.uniform(0.52, 0.72) * max_radius
            else:
                radius = rng.uniform(0.76, 0.94) * max_radius

            revolutions = rng.choice((0.5, 0.75, 1.0, 1.25))
            # Slightly calmer than the previous version. Large circles receive
            # more time so the servo does not turn them into jerky polygons.
            base_duration = fast(3.8, 2.15)
            duration = base_duration * (0.85 + 0.55 * (radius / max_radius))
            return Circle(here, center, radius=radius,
                          revolutions=revolutions, duration=duration,
                          direction=rng.choice((1, -1)), rng=rng)

        if kind == "retreat":
            return self._clamp(retreat(here, self.cat_hint,
                                       duration=fast(0.6, 0.28),
                                       distance=rng.uniform(0.4, 0.8), rng=rng))

        return Freeze(here, 0.5)

    def _catch_beat(self, here, intensity):
        """Let her win: slow, catchable wander, a long hold, then a startle."""
        if self.verbose:
            print("  -- catch beat --")
        near = clamp_to_box(here.pan + self.rng.uniform(-0.12, 0.12),
                            here.tilt + self.rng.uniform(-0.08, 0.08))
        self.r.play(Skitter(here, near, duration=2.2, amplitude=0.02, rng=self.rng))
        self.r.play(Freeze(self.r.current, duration=self.rng.uniform(1.8, 3.2), twitch=0.010))
        flee = self.cat_hint or self.r.current
        # 0.22s is a wish; _clamp turns it into the fastest the rig can do.
        self.r.play(self._clamp(retreat(self.r.current, flee, duration=0.22,
                                        distance=self.rng.uniform(0.5, 0.9),
                                        rng=self.rng)))

    # -- bout --------------------------------------------------------------
    def bout(self, seconds=BOUT_SECONDS, laser=None):
        start = time.monotonic()
        if laser:
            laser.on()
        self.r.play(self._dart(self.r.current, random_point(self.rng), duration=0.5))

        while not self.r.stop:
            elapsed = time.monotonic() - start
            if elapsed >= seconds:
                break
            u = min(1.0, elapsed / seconds)
            intensity = lerp_(INTENSITY_START, INTENSITY_END, u)

            if self.rng.random() < P_CATCH:
                self._catch_beat(self.r.current, intensity)
                self.last_kind = "catch"
            else:
                kind = self._pick()
                prim = self._make_move(kind, self.r.current, intensity)
                if self.verbose and not self.r.dry:
                    print(f"  {kind:10s} {prim.duration:4.2f}s  intensity {intensity:.2f}")
                self.r.play(prim)
                self.last_kind = kind

            if self.r.stop:
                break
            # Keep motion flowing most of the time. Brief pounce windows still
            # happen, but a move usually chains directly into the next move.
            pause_chance = lerp_(0.34, 0.16, intensity)
            if self.rng.random() < pause_chance:
                hold = lerp_(1.15, 0.18, intensity) * self.rng.uniform(0.65, 1.20)
                self.r.play(Freeze(self.r.current, duration=hold, twitch=0.009))

        if laser:
            laser.off()


def lerp_(a, b, w):
    return a + (b - a) * w


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry", action="store_true", help="no hardware; print the sequence")
    ap.add_argument("--once", action="store_true", help="one bout, then exit")
    ap.add_argument("--bout", type=float, default=BOUT_SECONDS, help="bout length (s)")
    ap.add_argument("--cooldown", type=float, default=COOLDOWN_SECONDS, help="rest between bouts (s)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--laser-brightness", type=float, default=LASER_BRIGHTNESS,
                    help="laser PWM duty cycle from 0.0 to 1.0 (default: 0.35)")
    ap.add_argument("--sweep", action="store_true",
                    help="slowly sweep both servos through the configured play box, then exit")
    args = ap.parse_args()
    if not 0.0 <= args.laser_brightness <= 1.0:
        ap.error("--laser-brightness must be between 0.0 and 1.0")

    cal = load_calibration()

    if args.dry:
        pca, laser = DummyPCA(), DummyLaser()
    else:
        pca = PCA9685()
        laser = DimmableLaser(LASER_GPIO_PIN, brightness=args.laser_brightness)
        laser.off()

    runner = Runner(pca, cal, dry=args.dry)
    chor = Choreographer(runner, rng=random.Random(args.seed))

    # Ctrl+C sets a flag rather than raising mid-primitive, so the current
    # move finishes its tick and we exit through the parking path.
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

    if args.sweep:
        # Walk the actual play-box edges so you can eyeball, with the laser
        # OFF, that the rig points where you expect. The endpoints come
        # straight from PAN_MIN/MAX and TILT_MIN/MAX, so this can never drift
        # out of sync with the safe box. TILT_MIN is the highest (closest to
        # horizontal) the dot should ever get — confirm it stays well below
        # eye level, and that TILT_MIN is DOWN, not up. If it sweeps up, flip
        # the tilt signs in primitives.py.
        pan_mid = (PAN_MIN + PAN_MAX) / 2.0     # = forward
        tilt_mid = (TILT_MIN + TILT_MAX) / 2.0
        try:
            print(f"Sweeping pan at tilt {tilt_mid:+.2f} "
                  f"({PAN_MIN:+.2f} .. {PAN_MAX:+.2f})...")
            for pan in (pan_mid, PAN_MIN, PAN_MAX, pan_mid):
                runner.glide_to(Aim(pan, tilt_mid), duration=1.2)
                time.sleep(0.25)
            print(f"Sweeping tilt at pan {pan_mid:+.2f} "
                  f"({TILT_MIN:+.2f}=highest .. {TILT_MAX:+.2f}=lowest)...")
            for tilt in (tilt_mid, TILT_MIN, TILT_MAX, tilt_mid):
                runner.glide_to(Aim(pan_mid, tilt), duration=1.2)
                time.sleep(0.25)
        finally:
            laser.off()
            if not args.dry:
                laser.close()
        print("Sweep complete.")
        return

    try:
        n = 0
        while not runner.stop:
            n += 1
            print(f"\n=== bout {n} ({args.bout:.0f}s) ===")
            chor.bout(seconds=args.bout, laser=laser)
            if args.once or runner.stop:
                break
            print(f"--- cooldown {args.cooldown/60:.1f} min (Ctrl+C to quit) ---")
            end = time.monotonic() + args.cooldown
            while time.monotonic() < end and not runner.stop:
                time.sleep(0.2)
    finally:
        laser.off()
        if not args.dry:
            try:
                runner.stop = False       # allow the park glide to run
                runner.glide_to(Aim(0.0, PARK_TILT), duration=0.7)
            except Exception:
                pass
            laser.close()
        print("Laser off, parked.")


if __name__ == "__main__":
    main()