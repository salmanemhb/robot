#!/usr/bin/env python3
"""
robot.py
--------
Drop this file in Code/Server/ on the Raspberry Pi, then run:

    cd Code/Server
    python3 robot.py

The robot will:
  1. Follow a black line using the three IR sensors
  2. Avoid obstacles using the ultrasonic sensor
  3. Pick up any object within reach using the servo clamp
"""

import os
import json
import subprocess
import time
import warnings


# =============================================================================
# PARAMETER MANAGER
# Reads / creates params.json to know PCB version and Pi version
# =============================================================================

class ParameterManager:
    PARAM_FILE = 'params.json'

    def __init__(self):
        self.file_path = self.PARAM_FILE
        if not self._exists() or not self._valid():
            self._setup()

    def _exists(self, path=None):
        return os.path.exists(path or self.file_path)

    def _valid(self, path=None):
        path = path or self.file_path
        if not self._exists(path):
            return False
        try:
            with open(path) as f:
                p = json.load(f)
            return (p.get('Pcb_Version') in [1, 2] and
                    p.get('Pi_Version')  in [1, 2])
        except Exception:
            return False

    def _read(self, key):
        if self._valid():
            with open(self.file_path) as f:
                return json.load(f).get(key)
        return None

    def _write(self, data):
        with open(self.file_path, 'w') as f:
            json.dump(data, f, indent=4)

    @staticmethod
    def _detect_pi():
        try:
            r = subprocess.run(
                ['cat', '/sys/firmware/devicetree/base/model'],
                capture_output=True, text=True)
            if r.returncode == 0 and 'Raspberry Pi 5' in r.stdout:
                return 2
        except Exception:
            pass
        return 1

    def _setup(self):
        print("No valid params.json found — first-time setup.")
        while True:
            try:
                pcb = int(input("  Enter PCB Version (1 or 2): "))
                if pcb in [1, 2]:
                    break
            except ValueError:
                pass
            print("  Please enter 1 or 2.")
        pi = self._detect_pi()
        self._write({'Pcb_Version': pcb, 'Pi_Version': pi})
        print(f"  Saved PCB={pcb}, Pi={pi}\n")

    def get_pcb_version(self):
        return self._read('Pcb_Version')

    def get_pi_version(self):
        return self._read('Pi_Version')


# =============================================================================
# INFRARED — three line sensors
# =============================================================================

class Infrared:
    """
    Reads three IR line sensors.
    read_all() returns a 3-bit integer:
      bit2 = left sensor, bit1 = centre, bit0 = right
      e.g. 0b010 (2) = only centre sensor on line → go straight
    """

    def __init__(self, pcb_version):
        from gpiozero import LineSensor
        if pcb_version == 1:
            pins = (16, 20, 21)
        else:
            pins = (16, 26, 21)
        self._s = [LineSensor(p) for p in pins]

    def _read(self, idx):
        return 1 if self._s[idx].value else 0

    def read_all(self):
        return (self._read(0) << 2) | (self._read(1) << 1) | self._read(2)

    def close(self):
        for s in self._s:
            s.close()


# =============================================================================
# ULTRASONIC — distance in cm
# =============================================================================

class _GpiozeroUltrasonic:
    def __init__(self, trigger, echo):
        from gpiozero import DistanceSensor, PWMSoftwareFallback
        warnings.filterwarnings('ignore', category=PWMSoftwareFallback)
        self._s = DistanceSensor(echo=echo, trigger=trigger, max_distance=3)

    def get_distance(self):
        try:
            return round(self._s.distance * 100, 1)
        except Exception:
            return -1

    def close(self):
        self._s.close()


class _LgpiodUltrasonic:
    def __init__(self, trigger, echo):
        import lgpio
        self._lg = lgpio
        self._trig = trigger
        self._echo = echo
        try:
            self._chip = lgpio.gpiochip_open(0)
        except Exception:
            self._chip = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self._chip, trigger)
        lgpio.gpio_claim_input(self._chip,  echo)

    def get_distance(self):
        try:
            lg = self._lg
            lg.gpio_write(self._chip, self._trig, 0)
            time.sleep(0.05)
            lg.gpio_write(self._chip, self._trig, 1)
            time.sleep(0.00001)
            lg.gpio_write(self._chip, self._trig, 0)
            deadline = time.time() + 1.0
            t0 = time.time()
            while lg.gpio_read(self._chip, self._echo) == 0:
                t0 = time.time()
                if t0 > deadline:
                    return -1
            t1 = time.time()
            while lg.gpio_read(self._chip, self._echo) == 1:
                t1 = time.time()
                if t1 > deadline:
                    return -1
            return round((t1 - t0) * 34300 / 2, 1)
        except Exception:
            return -1

    def close(self):
        if getattr(self, '_chip', None) is not None:
            try:
                self._lg.gpiochip_close(self._chip)
                self._chip = None
            except Exception:
                pass


class Ultrasonic:
    def __init__(self, pi_version, trigger=27, echo=22):
        if pi_version == 2:
            self._impl = _LgpiodUltrasonic(trigger, echo)
        else:
            self._impl = _GpiozeroUltrasonic(trigger, echo)

    def get_distance(self):
        return self._impl.get_distance()

    def close(self):
        self._impl.close()


# =============================================================================
# MOTORS — left and right tracks
# =============================================================================

class Motor:
    """
    setMotorModel(left, right) where values are in range -4095 .. +4095.
    Positive = forward, negative = backward.
    """

    def __init__(self):
        from gpiozero import Motor as _M
        self._left  = _M(24, 23)
        self._right = _M(5,  6)

    @staticmethod
    def _clamp(v):
        return max(-4095, min(4095, int(v)))

    @staticmethod
    def _drive(m, duty):
        if duty > 0:
            m.forward(duty / 4096)
        elif duty < 0:
            m.backward(-duty / 4096)
        else:
            m.stop()

    def setMotorModel(self, left, right):
        self._drive(self._left,  self._clamp(left))
        self._drive(self._right, self._clamp(right))

    def stop(self):
        self.setMotorModel(0, 0)

    def close(self):
        self._left.close()
        self._right.close()


# =============================================================================
# SERVO — two-channel arm + clamp
# =============================================================================

class _GpiozeroServo:
    """PCB v1 — soft PWM via gpiozero on GPIO 7, 8, 25."""

    def __init__(self):
        from gpiozero import AngularServo
        kw = dict(min_angle=0, max_angle=180,
                  min_pulse_width=0.0005, max_pulse_width=0.0025)
        self._s = {
            '0': AngularServo(7,  initial_angle=0, **kw),
            '1': AngularServo(8,  initial_angle=0, **kw),
            '2': AngularServo(25, initial_angle=0, **kw),
        }

    def setServoPwm(self, channel, angle):
        if channel in self._s:
            self._s[channel].angle = angle

    def stop(self):
        pass   # gpiozero servos don't need explicit stop


class _HardwareServo:
    """PCB v2 — hardware PWM on GPIO 12 (ch0) and GPIO 13 (ch1)."""

    def __init__(self):
        from rpi_hardware_pwm import HardwarePWM
        self._p = {
            '0': HardwarePWM(pwm_channel=0, hz=50, chip=0),
            '1': HardwarePWM(pwm_channel=1, hz=50, chip=0),
        }
        for p in self._p.values():
            p.start(0)

    @staticmethod
    def _duty(angle):
        return (angle / 180) * (12.5 - 2.5) + 2.5

    def setServoPwm(self, channel, angle):
        if channel in self._p:
            self._p[channel].change_duty_cycle(self._duty(angle))

    def stop(self):
        for p in self._p.values():
            try:
                p.stop()
            except Exception:
                pass


class Servo:
    # Valid angle ranges per channel
    _LIMITS = {'0': (90, 150), '1': (90, 150), '2': (0, 180)}

    def __init__(self, pcb_version, pi_version):
        if pcb_version == 2:
            self._impl = _HardwareServo()
        else:
            self._impl = _GpiozeroServo()
        # Park at resting position
        self.setServoAngle('0', 90)
        self.setServoAngle('1', 140)

    def setServoAngle(self, channel, angle):
        lo, hi = self._LIMITS.get(str(channel), (0, 180))
        angle = max(lo, min(hi, int(angle)))
        self._impl.setServoPwm(str(channel), angle)

    def setServoStop(self):
        self._impl.stop()


# =============================================================================
# ROBOT — main behaviour
# =============================================================================

class Robot:
    # ── Tuning constants ──────────────────────────────────────────────────────
    SPEED_FWD    = 1200   # normal forward speed
    SPEED_TURN   = 1500   # gentle turn
    SPEED_SHARP  = 2500   # sharp turn (outer wheel)
    AVOID_DIST   = 45.0   # cm — obstacle threshold in avoidance mode
    PICKUP_MAX   = 12.0   # cm — start pickup sequence if closer than this
    PICKUP_IDEAL_LO = 7.5 # cm — ideal grab distance (low)
    PICKUP_IDEAL_HI = 7.7 # cm — ideal grab distance (high)

    def __init__(self):
        print("=== Freenove Tank Robot ===")
        print("Initialising hardware…")
        params = ParameterManager()
        pcb = params.get_pcb_version()
        pi  = params.get_pi_version()
        print(f"  PCB version : {pcb}")
        print(f"  Pi  version : {'5' if pi == 2 else '4 or earlier'}")

        self.motor  = Motor()
        self.servo  = Servo(pcb, pi)
        self.ir     = Infrared(pcb)
        self.sonic  = Ultrasonic(pi)
        self._stop  = False
        print("Hardware ready.\n")

    # ── Clamp sequences ───────────────────────────────────────────────────────

    def _clamp_approach_and_grab(self):
        """
        Drive the robot to the exact grab distance, then sweep the servo
        arm down to pick the object up.
        """
        print("  [clamp] approaching object…")
        deadline = time.time() + 6.0
        grabbed  = False

        while time.time() < deadline and not self._stop:
            d = self.sonic.get_distance()
            if d < 0:
                break

            if d <= 5.0:
                self.motor.setMotorModel(-1200, -1200)   # too close, back off
            elif d < self.PICKUP_IDEAL_LO:
                self.motor.setMotorModel(-800, -800)
            elif self.PICKUP_IDEAL_LO <= d <= self.PICKUP_IDEAL_HI:
                self.motor.stop()
                # ── grab sequence ──────────────────────────────────────────
                print("  [clamp] grabbing…")
                for a in range(140, 89, -1):             # open / lower arm
                    self.servo.setServoAngle('1', a)
                    time.sleep(0.01)
                for a in range(90, 130):                 # extend arm forward
                    self.servo.setServoAngle('0', a)
                    time.sleep(0.01)
                for a in range(90, 140):                 # close / raise arm
                    self.servo.setServoAngle('1', a)
                    time.sleep(0.01)
                grabbed = True
                break
            elif d < 11.0:
                self.motor.setMotorModel(800, 800)
            else:
                self.motor.setMotorModel(1200, 1200)
            time.sleep(0.05)

        self.motor.stop()
        return grabbed

    def _clamp_release(self):
        """Lower the arm to deposit the object, then return to rest."""
        print("  [clamp] releasing…")
        self.motor.stop()
        for a in range(140, 89, -1):                     # open arm
            self.servo.setServoAngle('1', a)
            time.sleep(0.01)
        for a in range(130, 89, -1):                     # retract arm
            self.servo.setServoAngle('0', a)
            time.sleep(0.01)
        for a in range(90, 140):                         # close arm (rest)
            self.servo.setServoAngle('1', a)
            time.sleep(0.01)
        print("  [clamp] done.")

    # ── Single-step behaviours ────────────────────────────────────────────────

    def _step_line_follow(self, distance):
        """
        One tick of line-following.
        If an object is within pickup range, grab it, carry it aside,
        deposit it, then return to the original heading.

        IR bit layout (read_all returns 3-bit int):
          bit2=left, bit1=centre, bit0=right
          2 (010) → straight
          4 (100) → veer left
          6 (110) → sharp left
          1 (001) → veer right
          3 (011) → sharp right
          7 (111) → all sensors → stop
          0 (000) → no line
        """
        # ── Proximity pickup ─────────────────────────────────────────────────
        if 0 < distance <= self.PICKUP_MAX:
            self.motor.stop()
            grabbed = self._clamp_approach_and_grab()
            if grabbed:
                # Turn aside, deposit, turn back
                self.motor.setMotorModel(-1500, 1500)
                time.sleep(1.5)
                self.motor.stop()
                self._clamp_release()
                self.motor.setMotorModel(1500, -1500)
                time.sleep(1.4)
                self.motor.stop()
            return

        # ── Line following ───────────────────────────────────────────────────
        ir = self.ir.read_all()
        if   ir == 2: self.motor.setMotorModel( self.SPEED_FWD,   self.SPEED_FWD)
        elif ir == 4: self.motor.setMotorModel(-self.SPEED_TURN,  self.SPEED_SHARP)
        elif ir == 6: self.motor.setMotorModel(-2000,              4000)
        elif ir == 1: self.motor.setMotorModel( self.SPEED_SHARP, -self.SPEED_TURN)
        elif ir == 3: self.motor.setMotorModel( 4000,             -2000)
        elif ir == 7: self.motor.stop()
        # ir == 0 handled in the main loop

    def _step_avoid_obstacle(self):
        """
        One tick of pure obstacle avoidance (no line).
        Backs up and turns away from anything closer than AVOID_DIST.
        """
        d = self.sonic.get_distance()
        if 0 < d < self.AVOID_DIST:
            self.motor.setMotorModel(-1500, -1500)
            time.sleep(0.4)
            self.motor.setMotorModel(-1500,  1500)
            time.sleep(0.2)
        else:
            self.motor.setMotorModel(1500, 1500)
        time.sleep(0.2)

    # ── Main run loop ─────────────────────────────────────────────────────────

    def run(self):
        """
        Main loop.
        - Follows the black line while one is detected.
        - Falls back to obstacle-avoidance mode if no line is found for 2 s.
        - Returns to line-following as soon as the line is picked up again.
        """
        print("Robot running — press Ctrl+C to stop.\n")
        no_line_since = None

        try:
            while not self._stop:
                ir       = self.ir.read_all()
                distance = self.sonic.get_distance()

                if ir == 0:
                    # No line under any sensor
                    if no_line_since is None:
                        no_line_since = time.time()

                    if time.time() - no_line_since > 2.0:
                        # Line lost for > 2 s → switch to obstacle avoidance
                        self._step_avoid_obstacle()
                    else:
                        # Briefly lost — creep forward hoping to re-acquire
                        self.motor.setMotorModel(self.SPEED_FWD, self.SPEED_FWD)
                        time.sleep(0.05)
                else:
                    no_line_since = None
                    self._step_line_follow(distance)

        except KeyboardInterrupt:
            print("\nCtrl+C received — shutting down.")
        finally:
            self.close()

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def close(self):
        self._stop = True
        self.motor.stop()
        self.servo.setServoStop()
        self.motor.close()
        self.ir.close()
        self.sonic.close()
        print("Robot stopped cleanly.")


# =============================================================================
if __name__ == '__main__':
    robot = Robot()
    robot.run()
