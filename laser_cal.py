#!/usr/bin/env python3
"""
servo_cal.py — interactive PCA9685 servo calibration tool (SSH-compatible).

Finds the real min/max/center pulse widths for each servo (the physical
limits, not a generic 0.5-2.5ms guess) and saves them to servo_cal.json.
servo_wasd.py automatically loads that file if it exists.

Input comes from stdin (the terminal / PTY), so this works identically
whether you run it in a local terminal on the Pi or over SSH. No evdev,
no device grabbing, no 'input' group membership, and no sudo required —
the keys you press in the SSH session arrive on this process's stdin.

Controls:
  1 / 2        →  select channel (1 = tilt/up, 2 = pan/down)
  Up / Down    →  nudge pulse width (coarse step)
  Left / Right →  nudge pulse width (fine step)
  [            →  mark current position as this channel's MIN
  ]            →  mark current position as this channel's MAX
  c            →  mark current position as this channel's CENTER
  s            →  save all marked values to servo_cal.json
  q            →  quit (prompts to save if there are unsaved changes)

Move slowly and listen/watch for the servo straining or buzzing at the
limit — that's the mechanical stop. Back off slightly (~0.02-0.03ms) from
the point where it strains; that's your real MIN/MAX, not the strain point
itself.

Requirements:
  pip install smbus2
"""

import json
import os
import select
import sys
import termios
import time
import tty
import smbus2

I2C_BUS          = 1
I2C_ADDR         = 0x40

PCA9685_MODE1    = 0x00
PCA9685_PRESCALE = 0xFE
LED0_ON_L        = 0x06

SERVO_UP_CH      = 0
SERVO_DOWN_CH    = 1

PWM_FREQ         = 60      # Hz

COARSE_STEP_MS   = 0.05
FINE_STEP_MS     = 0.01

# Safety bounds for manual jogging. These are NOT hardware limits — at 60Hz
# the period is ~16.67ms, so there's headroom well past typical servo range.
# Raise PULSE_MAX_MS if you need more travel, but walk it up gradually and
# watch/listen for straining or buzzing rather than jumping straight to a
# large number.
PULSE_MIN_MS     = 0.2
PULSE_MAX_MS     = 4.0

CAL_FILE         = "servo_cal.json"

CHANNEL_NAMES = {SERVO_UP_CH: "tilt (channel 0)", SERVO_DOWN_CH: "pan (channel 1)"}


class PCA9685:
    def __init__(self, bus: int = I2C_BUS, addr: int = I2C_ADDR):
        self.bus = smbus2.SMBus(bus)
        self.addr = addr
        self._write_reg(PCA9685_MODE1, 0x80)
        time.sleep(0.01)
        self.set_pwm_freq(1000)
        self.set_pwm_freq(PWM_FREQ)

    def _write_reg(self, reg: int, value: int):
        self.bus.write_byte_data(self.addr, reg, value)

    def _read_reg(self, reg: int) -> int:
        return self.bus.read_byte_data(self.addr, reg)

    def set_pwm_freq(self, freq_hz: float):
        prescale_val = 25000000.0
        prescale_val /= 4096.0
        prescale_val /= float(freq_hz)
        prescale_val -= 1.0
        prescale_val *= 0.8449
        prescale = int(round(prescale_val))

        old_mode = self._read_reg(PCA9685_MODE1)
        self._write_reg(PCA9685_MODE1, (old_mode & 0x7F) | 0x10)
        self._write_reg(PCA9685_PRESCALE, prescale)
        self._write_reg(PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self._write_reg(PCA9685_MODE1, old_mode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int):
        base = LED0_ON_L + 4 * channel
        self._write_reg(base, on & 0xFF)
        self._write_reg(base + 1, (on >> 8) & 0xFF)
        self._write_reg(base + 2, off & 0xFF)
        self._write_reg(base + 3, (off >> 8) & 0xFF)

    def set_pulse_ms(self, channel: int, pulse_ms: float):
        period_ms = 1000.0 / PWM_FREQ
        off_count = int((pulse_ms / period_ms) * 4096.0)
        off_count = max(0, min(4095, off_count))
        self.set_pwm(channel, 0, off_count)


def read_key(fd):
    """Blocking read of one logical keypress from a stdin fd in cbreak mode.

    Returns a normalized token: 'UP' / 'DOWN' / 'LEFT' / 'RIGHT' / 'ESC' for
    special keys, the literal character for everything else, or None for a
    byte we couldn't decode / an incomplete sequence.

    Arrow keys arrive as an escape sequence over the terminal:
        ESC [ A/B/C/D      (normal cursor mode)
        ESC O A/B/C/D      (application cursor mode — some terminals/tmux)
    We read the ESC, then use select() with a short timeout to tell a real
    escape sequence apart from a bare Escape keypress (which has no bytes
    following it).
    """
    b = os.read(fd, 1)
    if not b:
        return None
    if b != b"\x1b":
        try:
            return b.decode("utf-8")
        except UnicodeDecodeError:
            return None

    # Got ESC. If nothing follows almost immediately, it was a bare Escape.
    if not select.select([fd], [], [], 0.05)[0]:
        return "ESC"

    rest = os.read(fd, 2)
    if rest[:1] not in (b"[", b"O"):
        return "ESC"

    final = rest[1:2]
    if not final:
        # '[' and the final byte arrived in separate reads — grab it.
        if select.select([fd], [], [], 0.05)[0]:
            final = os.read(fd, 1)

    return {b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT"}.get(final)


def load_existing():
    try:
        with open(CAL_FILE) as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


def main():
    if not sys.stdin.isatty():
        print("stdin is not a terminal — run this interactively (a normal "
              "SSH session or local terminal), not with piped/redirected input.")
        sys.exit(1)

    print("Initializing PCA9685...")
    pca = PCA9685()

    cal = load_existing()
    for ch in (SERVO_UP_CH, SERVO_DOWN_CH):
        key = str(ch)
        if key not in cal:
            cal[key] = {"min_ms": 0.5, "max_ms": 2.5, "center_ms": 1.5}

    active_ch = SERVO_UP_CH
    pulse_ms = cal[str(active_ch)]["center_ms"]
    pca.set_pulse_ms(active_ch, pulse_ms)

    stdin_fd = sys.stdin.fileno()
    old_term_settings = termios.tcgetattr(stdin_fd)

    # cbreak: non-canonical, no line buffering, but ISIG stays on so Ctrl-C
    # still raises KeyboardInterrupt (caught below). Then explicitly drop
    # ECHO so keystrokes don't print over the status line, and flush anything
    # that queued on stdin before we switched modes so it can't be replayed
    # into the shell on exit.
    tty.setcbreak(stdin_fd)
    no_echo = termios.tcgetattr(stdin_fd)
    no_echo[3] &= ~termios.ECHO  # lflags
    termios.tcsetattr(stdin_fd, termios.TCSADRAIN, no_echo)
    termios.tcflush(stdin_fd, termios.TCIFLUSH)

    unsaved = False

    def status():
        print(f"[{CHANNEL_NAMES[active_ch]}] pulse = {pulse_ms:.3f} ms   "
              f"(marked min={cal[str(active_ch)]['min_ms']:.3f}, "
              f"max={cal[str(active_ch)]['max_ms']:.3f}, "
              f"center={cal[str(active_ch)]['center_ms']:.3f})")

    print("\n1/2 = select channel | Up/Down = coarse | Left/Right = fine")
    print("[ = mark min | ] = mark max | c = mark center | s = save | q = quit\n")
    status()

    try:
        while True:
            key = read_key(stdin_fd)
            if key is None:
                continue

            # Normalize letter case so Shift/CapsLock don't break the marks.
            cmd = key.lower() if len(key) == 1 and key.isalpha() else key

            if cmd == "1":
                active_ch = SERVO_UP_CH
                pulse_ms = cal[str(active_ch)]["center_ms"]
                pca.set_pulse_ms(active_ch, pulse_ms)
                status()

            elif cmd == "2":
                active_ch = SERVO_DOWN_CH
                pulse_ms = cal[str(active_ch)]["center_ms"]
                pca.set_pulse_ms(active_ch, pulse_ms)
                status()

            elif cmd == "UP":
                pulse_ms = min(PULSE_MAX_MS, pulse_ms + COARSE_STEP_MS)
                pca.set_pulse_ms(active_ch, pulse_ms)
                status()

            elif cmd == "DOWN":
                pulse_ms = max(PULSE_MIN_MS, pulse_ms - COARSE_STEP_MS)
                pca.set_pulse_ms(active_ch, pulse_ms)
                status()

            elif cmd == "RIGHT":
                pulse_ms = min(PULSE_MAX_MS, pulse_ms + FINE_STEP_MS)
                pca.set_pulse_ms(active_ch, pulse_ms)
                status()

            elif cmd == "LEFT":
                pulse_ms = max(PULSE_MIN_MS, pulse_ms - FINE_STEP_MS)
                pca.set_pulse_ms(active_ch, pulse_ms)
                status()

            elif cmd == "[":
                cal[str(active_ch)]["min_ms"] = round(pulse_ms, 3)
                unsaved = True
                print(f"  → marked MIN for {CHANNEL_NAMES[active_ch]}: {pulse_ms:.3f} ms")
                status()

            elif cmd == "]":
                cal[str(active_ch)]["max_ms"] = round(pulse_ms, 3)
                unsaved = True
                print(f"  → marked MAX for {CHANNEL_NAMES[active_ch]}: {pulse_ms:.3f} ms")
                status()

            elif cmd == "c":
                cal[str(active_ch)]["center_ms"] = round(pulse_ms, 3)
                unsaved = True
                print(f"  → marked CENTER for {CHANNEL_NAMES[active_ch]}: {pulse_ms:.3f} ms")
                status()

            elif cmd == "s":
                with open(CAL_FILE, "w") as f:
                    json.dump(cal, f, indent=2)
                unsaved = False
                print(f"  → saved to {CAL_FILE}")

            elif cmd == "q":
                if unsaved == "confirm":
                    break
                elif unsaved:
                    print("  You have unsaved changes. Press 's' to save, or 'q' again to quit without saving.")
                    unsaved = "confirm"
                else:
                    break

    except KeyboardInterrupt:
        pass
    finally:
        try:
            # Discard any buffered/leaked input before handing control back to
            # the shell so it doesn't get dumped in all at once, then restore.
            termios.tcflush(stdin_fd, termios.TCIFLUSH)
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)
        except Exception:
            pass
        print("\nFinal calibration:")
        print(json.dumps(cal, indent=2))
        if unsaved and unsaved != "confirm":
            print(f"NOTE: you have unsaved changes — run again or edit {CAL_FILE} manually if needed.")


if __name__ == "__main__":
    main()