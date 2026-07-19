#!/usr/bin/env python3
"""
primitives.py — time-parameterized laser motion.

A Primitive is a pure function of time:  p.at(t) -> Aim(pan, tilt)

Coordinates are normalized to a PLAY BOX: (-1..1, -1..1) over the patch of
floor you want to play on. Milliseconds only appear at the very edge, in
norm_to_pulse().

RULE: a primitive may START with nonzero velocity (that's the flick that
triggers a pounce) but must END at zero velocity. Chaining is then
C1-continuous with no blending logic.

No hardware needed:
    python primitives.py --demo
"""

import argparse
import json
import math
import random
from collections import namedtuple

Aim = namedtuple("Aim", "pan tilt")

CAL_FILE = "servo_cal.json"

SERVO_UP_CH = 0    # tilt
SERVO_DOWN_CH = 1  # pan

UPDATE_HZ = 50.0   # PWM refresh is 60Hz; above that buys nothing
DT = 1.0 / UPDATE_HZ

# ---------------------------------------------------------------------------
# Play box — the safe, useful envelope. TUNE THESE FIRST.
# ---------------------------------------------------------------------------
# Normalized fractions of each servo's calibrated travel. The servos are now
# calibrated so that:
#     normalized  0  -> center = STRAIGHT FORWARD (horizontal)
#     normalized +1  -> max_ms = +90 deg
#     normalized -1  -> min_ms = -90 deg
# i.e. a normalized value is directly (fraction of 90 deg) off of forward.
#
# PAN is symmetric about forward and is safe in either direction — it just
# swings the dot left/right across the floor.
#
# TILT IS THE EYE-SAFETY AXIS. Center is now HORIZONTAL (~eye level), so
# tilt=0 points the beam straight out at a person's eyes. The entire tilt
# play box therefore lives on the DOWNWARD side of center, with margin, so the
# beam never rises toward eye level. TILT_MIN is that "never point higher than
# this" limit — an eye-safety bound, not a stylistic one.
#
# Sign convention: +tilt = downward (same as before). VERIFY THIS on your rig
# with `--sweep` before running near anyone. If the dot sweeps UP instead of
# down, flip the tilt signs here (and swap TILT_MIN/TILT_MAX) — not in the
# math. Nothing enforces any of this at runtime; it's open-loop. Check by hand.
PAN_MIN, PAN_MAX = -0.65, -0.2     # ~+-58 deg about forward
TILT_MIN, TILT_MAX = -0.55, 0.55    # ~27 deg to ~77 deg BELOW horizontal (down only)

PAN_SPAN = PAN_MAX - PAN_MIN
TILT_SPAN = TILT_MAX - TILT_MIN
ASPECT = TILT_SPAN / PAN_SPAN      # keeps circles from looking squashed

# Datasheet-ish for an SG90 under load. Command past this and the servo
# saturates: it ignores your easing curve and does its own ramp.
SERVO_MAX_DEG_PER_S = 300.0
# Full calibrated span (min_ms..max_ms) in degrees. With the -90..+90 deg
# calibration above, that span is 180 deg. If your true min..max travel is
# something else (e.g. +-45 deg -> 90 total), set this to match, or the servo
# saturation guard (min_dart_duration / peak_speed) will be wrong.
SERVO_TRAVEL_DEG = 180.0


# ---------------------------------------------------------------------------
# Easing — u in [0,1] -> [0,1]
# ---------------------------------------------------------------------------
def linear(u):
    return u


def smoothstep(u):
    """C1: zero velocity at both ends."""
    return u * u * (3.0 - 2.0 * u)


def smootherstep(u):
    """C2: zero velocity AND acceleration at both ends."""
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def ease_out_cubic(u):
    """Explosive start, decelerating arrival. What a bug does."""
    return 1.0 - (1.0 - u) ** 3


def ease_out_expo(u):
    """More violent start than cubic. For a startle."""
    return 1.0 if u >= 1.0 else 1.0 - pow(2.0, -10.0 * u)


def window(u, ramp=0.2):
    """Envelope that is 0 with zero slope at both ends. Multiply noise by
    this and the noise can't break the end-at-rest rule."""
    return smoothstep(min(1.0, u / ramp)) * smoothstep(min(1.0, (1.0 - u) / ramp))


def lerp(a, b, w):
    return a + (b - a) * w


def clamp_to_box(pan, tilt):
    return Aim(max(PAN_MIN, min(PAN_MAX, pan)),
               max(TILT_MIN, min(TILT_MAX, tilt)))


def random_point(rng=random, margin=0.0):
    return Aim(rng.uniform(PAN_MIN + margin, PAN_MAX - margin),
               rng.uniform(TILT_MIN + margin, TILT_MAX - margin))


def ray_box_max(p, dpan, dtilt):
    """How far can we travel from p along (dpan, dtilt) and stay in the box?"""
    tmax = float("inf")
    for val, dv, lo, hi in ((p.pan, dpan, PAN_MIN, PAN_MAX),
                            (p.tilt, dtilt, TILT_MIN, TILT_MAX)):
        if abs(dv) < 1e-9:
            continue
        tmax = min(tmax, max((lo - val) / dv, (hi - val) / dv))
    return 0.0 if tmax == float("inf") else max(0.0, tmax)


# ---------------------------------------------------------------------------
# Primitives
# ---------------------------------------------------------------------------
class Primitive:
    duration = 0.0

    def at(self, t):
        raise NotImplementedError

    def end(self):
        return self.at(self.duration)


class Freeze(Primitive):
    """Hold still. The most important primitive in the file.

    Cats pounce on prey that STOPPED, not on prey that's moving. The twitch
    keeps a stopped dot alive rather than dead — two incommensurable sine
    products so it never visibly loops.
    """

    def __init__(self, at_aim, duration, twitch=0.006):
        self.aim = clamp_to_box(*at_aim)
        self.duration = duration
        self.twitch = twitch

    def at(self, t):
        if self.twitch <= 0:
            return self.aim
        fade = smoothstep(min(1.0, t / 0.25))
        j_pan = math.sin(t * 11.0) * math.sin(t * 3.7) * self.twitch * fade
        j_tilt = math.sin(t * 9.3 + 1.7) * math.sin(t * 4.1) * self.twitch * fade
        return clamp_to_box(self.aim.pan + j_pan, self.aim.tilt + j_tilt)


class Dart(Primitive):
    """Fast run from start to target, decelerating into the stop.

    ease_out_cubic: leaves hard, settles soft. `bow` arcs the path slightly
    so it doesn't read as a ruler-straight machine move — sine envelope, so
    zero deflection at both ends.
    """

    def __init__(self, start, target, duration=0.35, bow=0.0):
        self.start = clamp_to_box(*start)
        self.target = clamp_to_box(*target)
        self.duration = max(0.05, duration)
        self.bow = bow

    def at(self, t):
        u = min(1.0, t / self.duration)
        w = ease_out_cubic(u)
        pan = lerp(self.start.pan, self.target.pan, w)
        tilt = lerp(self.start.tilt, self.target.tilt, w)
        if self.bow:
            dx = self.target.pan - self.start.pan
            dy = self.target.tilt - self.start.tilt
            n = math.hypot(dx, dy) or 1.0
            k = math.sin(math.pi * w) * self.bow
            pan += (-dy / n) * k
            tilt += (dx / n) * k
        return clamp_to_box(pan, tilt)


class Skitter(Primitive):
    """Nervous drift — a bug that can't commit to a direction.

    Slow smoothstep drift toward target, plus summed sines at incommensurable
    frequencies, all under a window() envelope so it lands at rest.
    """

    def __init__(self, start, target, duration=1.2, amplitude=0.05, rng=random):
        self.start = clamp_to_box(*start)
        self.target = clamp_to_box(*target)
        self.duration = max(0.2, duration)
        self.amplitude = amplitude
        self.f = [(rng.uniform(4.0, 9.0), rng.uniform(0, 2 * math.pi)) for _ in range(3)]
        self.g = [(rng.uniform(4.0, 9.0), rng.uniform(0, 2 * math.pi)) for _ in range(3)]

    @staticmethod
    def _noise(terms, t):
        return sum(math.sin(t * f + p) / (i + 1) for i, (f, p) in enumerate(terms))

    def at(self, t):
        u = min(1.0, t / self.duration)
        w = smoothstep(u)
        env = window(u) * self.amplitude
        pan = lerp(self.start.pan, self.target.pan, w) + self._noise(self.f, t) * env
        tilt = lerp(self.start.tilt, self.target.tilt, w) + self._noise(self.g, t) * env * ASPECT
        return clamp_to_box(pan, tilt)


class BackAndForth(Primitive):
    """Repeated side-to-side chase between two safe points.

    Each leg uses smoothstep, so the servo reverses cleanly instead of being
    hit with an instantaneous direction change. The primitive ends at rest.
    """

    def __init__(self, start, target, legs=4, leg_duration=0.55, bow=0.03):
        self.start = clamp_to_box(*start)
        self.target = clamp_to_box(*target)
        self.legs = max(2, int(legs))
        self.leg_duration = max(0.18, float(leg_duration))
        self.duration = self.legs * self.leg_duration
        self.bow = bow

    def at(self, t):
        t = max(0.0, min(self.duration, t))
        leg = min(self.legs - 1, int(t / self.leg_duration))
        local_t = t - leg * self.leg_duration
        u = min(1.0, local_t / self.leg_duration)
        w = smoothstep(u)

        a, b = ((self.start, self.target) if leg % 2 == 0
                else (self.target, self.start))
        pan = lerp(a.pan, b.pan, w)
        tilt = lerp(a.tilt, b.tilt, w)

        if self.bow:
            dx = b.pan - a.pan
            dy = b.tilt - a.tilt
            n = math.hypot(dx, dy) or 1.0
            k = math.sin(math.pi * w) * self.bow
            pan += (-dy / n) * k
            tilt += (dx / n) * k

        return clamp_to_box(pan, tilt)


class Circle(Primitive):
    """Arc around a center. Blends from wherever you are onto the ellipse, so
    it's continuous no matter where the last primitive left off.

    Radius eases in (smoothstep), angle sweeps with smootherstep — both hit
    zero velocity at the end. Tight + fast is where you'll blow the servo
    velocity budget first; check --demo output.
    """

    def __init__(self, start, center, radius, revolutions=1.0, duration=2.0,
                 direction=1, rng=random):
        self.start = clamp_to_box(*start)
        self.center = clamp_to_box(*center)
        self.rx = radius
        self.ry = radius * ASPECT
        self.revolutions = revolutions
        self.duration = max(0.3, duration)
        self.direction = direction

        self.off0 = (self.start.pan - self.center.pan,
                     self.start.tilt - self.center.tilt)
        if math.hypot(*self.off0) < 1e-6:
            self.a0 = rng.uniform(0, 2 * math.pi)
        else:
            self.a0 = math.atan2(self.off0[1] / (self.ry or 1e-6),
                                 self.off0[0] / (self.rx or 1e-6))

    def at(self, t):
        u = min(1.0, t / self.duration)
        wa = smootherstep(u)
        w = smoothstep(u)
        ang = self.a0 + self.direction * 2.0 * math.pi * self.revolutions * wa
        ex = self.rx * math.cos(ang)
        ey = self.ry * math.sin(ang)
        pan = self.center.pan + lerp(self.off0[0], ex, w)
        tilt = self.center.tilt + lerp(self.off0[1], ey, w)
        return clamp_to_box(pan, tilt)


class Sequence(Primitive):
    """Chain primitives. Each ends at rest, so no blending needed."""

    def __init__(self, parts):
        self.parts = list(parts)
        self.duration = sum(p.duration for p in self.parts)

    def at(self, t):
        for p in self.parts:
            if t <= p.duration:
                return p.at(t)
            t -= p.duration
        return self.parts[-1].end()


def retreat(start, flee_from, duration=0.4, distance=0.6, bow=0.06, rng=random):
    """Run AWAY from a point. This is what makes it prey instead of a dot.

    If straight-away hits a wall immediately (cat has the dot cornered), fan
    out through progressively wider angles and take the first direction with
    real room. Returns a Dart.
    """
    dx = start.pan - flee_from.pan
    dy = start.tilt - flee_from.tilt
    n = math.hypot(dx, dy)
    if n < 1e-6:
        a = rng.uniform(0, 2 * math.pi)
        dx, dy = math.cos(a), math.sin(a)
    else:
        dx, dy = dx / n, dy / n

    best = None
    for deg in (0, 25, -25, 50, -50, 80, -80, 110, -110):
        r = math.radians(deg)
        cx = dx * math.cos(r) - dy * math.sin(r)
        cy = dx * math.sin(r) + dy * math.cos(r)
        room = ray_box_max(start, cx, cy)
        if best is None or room > best[0]:
            best = (room, cx, cy)
        if room >= distance:
            break

    room, cx, cy = best
    d = min(distance, room)
    target = Aim(start.pan + cx * d, start.tilt + cy * d)
    return Dart(start, target, duration=duration, bow=rng.uniform(-bow, bow))


# ---------------------------------------------------------------------------
# Calibration / output mapping
# ---------------------------------------------------------------------------
DEFAULT_CAL = {"0": {"min_ms": 0.5, "max_ms": 2.5, "center_ms": 1.5},
               "1": {"min_ms": 0.5, "max_ms": 2.5, "center_ms": 1.5}}


def load_calibration(path=CAL_FILE):
    try:
        with open(path) as f:
            cal = json.load(f)
        for k, v in DEFAULT_CAL.items():
            cal.setdefault(k, v)
        print(f"Loaded servo calibration from {path}")
        return cal
    except FileNotFoundError:
        print(f"No {path} — using 0.5-2.5ms defaults. Run servo_cal.py.")
        return {k: dict(v) for k, v in DEFAULT_CAL.items()}


def norm_to_pulse(n, cal_ch):
    """Normalized -1..1 -> pulse ms, honoring an asymmetric calibration."""
    c = cal_ch["center_ms"]
    n = max(-1.0, min(1.0, n))
    return c + n * ((cal_ch["max_ms"] - c) if n >= 0 else (c - cal_ch["min_ms"]))


def norm_to_deg(n, cal_ch):
    span = cal_ch["max_ms"] - cal_ch["min_ms"]
    return (norm_to_pulse(n, cal_ch) - cal_ch["min_ms"]) / span * SERVO_TRAVEL_DEG


def min_dart_duration(start, target, cal, margin=1.65):
    """Shortest duration a Dart can take without outrunning the servo.

    ease_out_cubic is 1-(1-u)^3, whose derivative at u=0 is exactly 3. So a
    Dart's peak speed is always 3*delta/duration — no sampling needed. Invert
    that against SERVO_MAX_DEG_PER_S and you get the floor. Ask for less and
    the servo saturates: it ignores the curve and does its own ramp.
    """
    pan_cal, tilt_cal = cal[str(SERVO_DOWN_CH)], cal[str(SERVO_UP_CH)]
    d_pan = abs(norm_to_deg(target.pan, pan_cal) - norm_to_deg(start.pan, pan_cal))
    d_tilt = abs(norm_to_deg(target.tilt, tilt_cal) - norm_to_deg(start.tilt, tilt_cal))
    delta = max(d_pan, d_tilt)
    return 3.0 * delta / SERVO_MAX_DEG_PER_S * margin


def peak_speed(prim, cal, steps=None):
    """Peak commanded deg/s for pan and tilt. Pure — no hardware, no sleeping."""
    pan_cal, tilt_cal = cal[str(SERVO_DOWN_CH)], cal[str(SERVO_UP_CH)]
    steps = steps or max(2, int(prim.duration * UPDATE_HZ))
    prev = prim.at(0.0)
    pk_p = pk_t = 0.0
    for i in range(1, steps + 1):
        a = prim.at(prim.duration * i / steps)
        pk_p = max(pk_p, abs(norm_to_deg(a.pan, pan_cal) - norm_to_deg(prev.pan, pan_cal)) / DT)
        pk_t = max(pk_t, abs(norm_to_deg(a.tilt, tilt_cal) - norm_to_deg(prev.tilt, tilt_cal)) / DT)
        prev = a
    return pk_p, pk_t


def demo():
    cal = {k: dict(v) for k, v in DEFAULT_CAL.items()}
    rng = random.Random(7)
    here = Aim(0.0, 0.5)
    print(f"{'primitive':12s} {'dur':>5s}  {'pan d/s':>8s} {'tilt d/s':>9s}")
    for _ in range(2):
        for make in (
            lambda h: Dart(h, random_point(rng), rng.uniform(0.25, 0.5), rng.uniform(-0.08, 0.08)),
            lambda h: Freeze(h, rng.uniform(0.6, 2.5)),
            lambda h: Skitter(h, random_point(rng), 1.4, 0.05, rng),
            lambda h: Freeze(h, 1.0),
            lambda h: Circle(h, random_point(rng, 0.25), 0.22, 1.0, 2.0, rng.choice((1, -1)), rng),
            lambda h: retreat(h, random_point(rng), rng=rng),
        ):
            p = make(here)
            pp, pt = peak_speed(p, cal)
            flag = "  <-- SATURATES" if max(pp, pt) > SERVO_MAX_DEG_PER_S else ""
            print(f"{p.__class__.__name__:12s} {p.duration:5.2f}  {pp:8.1f} {pt:9.1f}{flag}")
            here = p.end()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="dry-run, report velocities")
    if ap.parse_args().demo:
        demo()
    else:
        print(__doc__)