#!/usr/bin/env python3
"""
robot.py
--------
Run from Code/Server/ on the Raspberry Pi:

    cd ~/Freenove_Tank_Robot_Kit_for_Raspberry_Pi/Code/Server
    python3 robot.py

Combines (exactly from Freenove source files):
  parameter.py · infrared.py · ultrasonic.py · motor.py · servo.py · car.py

Plus red ball detection from Red_Ball_Detection.py.

Behaviour priority:
  1. Red ball visible  → steer toward it; pick up when within range
  2. Obstacle 12-45 cm → back up and turn away
  3. Otherwise         → follow the black line
"""

import os
import json
import subprocess
import time
import warnings
import cv2
import numpy as np

# =============================================================================
# parameter.py — ParameterManager
# =============================================================================

class ParameterManager:
    PARAM_FILE = 'params.json'

    def __init__(self):
        self.file_path = self.PARAM_FILE
        if self.file_exists() == False or self.validate_params() == False:
            self.deal_with_param()

    def file_exists(self, file_path=None):
        file_path = file_path or self.file_path
        return os.path.exists(file_path)

    def validate_params(self, file_path=None):
        file_path = file_path or self.file_path
        if not self.file_exists(file_path):
            return False
        try:
            with open(file_path, 'r') as file:
                params = json.load(file)
                return ('Pcb_Version' in params and params['Pcb_Version'] in [1, 2]) and \
                       ('Pi_Version' in params and params['Pi_Version'] in [1, 2])
        except json.JSONDecodeError:
            print("Error decoding JSON file.")
            return False
        except Exception as e:
            print(f"Error reading file: {e}")
            return False

    def get_param(self, param_name, file_path=None):
        file_path = file_path or self.file_path
        if self.validate_params(file_path):
            with open(file_path, 'r') as file:
                params = json.load(file)
                return params.get(param_name)
        return None

    def set_param(self, param_name, value, file_path=None):
        file_path = file_path or self.file_path
        params = {}
        if self.file_exists(file_path):
            with open(file_path, 'r') as file:
                params = json.load(file)
        params[param_name] = value
        with open(file_path, 'w') as file:
            json.dump(params, file, indent=4)

    def delete_param_file(self, file_path=None):
        file_path = file_path or self.file_path
        if self.file_exists(file_path):
            os.remove(file_path)
            print(f"Deleted {file_path}")
        else:
            print(f"File {file_path} does not exist")

    def create_param_file(self, file_path=None):
        file_path = file_path or self.file_path
        default_params = {
            'Pcb_Version': 2,
            'Pi_Version': self.get_raspberry_pi_version()
        }
        with open(file_path, 'w') as file:
            json.dump(default_params, file, indent=4)

    def get_raspberry_pi_version(self):
        try:
            result = subprocess.run(['cat', '/sys/firmware/devicetree/base/model'],
                                    capture_output=True, text=True)
            if result.returncode == 0:
                model = result.stdout.strip()
                if "Raspberry Pi 5" in model:
                    return 2
                else:
                    return 1
            else:
                print("Failed to get Raspberry Pi model information.")
                return 1
        except Exception as e:
            print(f"Error getting Raspberry Pi version: {e}")
            return 1

    def deal_with_param(self):
        if not self.file_exists() or not self.validate_params():
            print(f"Parameter file {self.PARAM_FILE} does not exist or contains invalid parameters.")
            user_input_required = True
        else:
            user_choice = input("Do you want to re-enter the hardware versions? (yes/no): ").strip().lower()
            user_input_required = user_choice == 'yes'

        if user_input_required:
            print("Please enter the hardware versions.")
            while True:
                try:
                    pcb_version = int(input("Enter PCB Version (1 or 2): "))
                    if pcb_version in [1, 2]:
                        break
                    else:
                        print("Invalid PCB Version. Please enter 1 or 2.")
                except ValueError:
                    print("Invalid input. Please enter a number.")
            pi_version = self.get_raspberry_pi_version()
            self.create_param_file()
            self.set_param('Pcb_Version', pcb_version)
            self.set_param('Pi_Version', pi_version)
        else:
            print("Do not modify the hardware version. Skipping...")

    def get_pcb_version(self):
        return self.get_param('Pcb_Version')

    def get_pi_version(self):
        return self.get_param('Pi_Version')


# =============================================================================
# infrared.py — Infrared
# =============================================================================

class Infrared:
    def __init__(self):
        from gpiozero import LineSensor
        self.param = ParameterManager()
        self.pcb_version = self.param.get_pcb_version()
        self.pi_version = self.param.get_raspberry_pi_version()

        if self.pcb_version == 1:
            self.IR01 = 16
            self.IR02 = 20
            self.IR03 = 21
        elif self.pcb_version == 2:
            self.IR01 = 16
            self.IR02 = 26
            self.IR03 = 21

        self.IR01_sensor = LineSensor(self.IR01)
        self.IR02_sensor = LineSensor(self.IR02)
        self.IR03_sensor = LineSensor(self.IR03)

    def read_one_infrared(self, channel):
        if channel == 1:
            return 1 if self.IR01_sensor.value else 0
        elif channel == 2:
            return 1 if self.IR02_sensor.value else 0
        elif channel == 3:
            return 1 if self.IR03_sensor.value else 0

    def read_all_infrared(self):
        return (self.read_one_infrared(1) << 2) | (self.read_one_infrared(2) << 1) | self.read_one_infrared(3)

    def close(self):
        self.IR01_sensor.close()
        self.IR02_sensor.close()
        self.IR03_sensor.close()


# =============================================================================
# ultrasonic.py — gpiozero_ultrasonic, lgpiod_ultrasonic, Ultrasonic
# =============================================================================

class gpiozero_ultrasonic:
    def __init__(self, trigger_pin=27, echo_pin=22):
        try:
            from gpiozero import DistanceSensor, PWMSoftwareFallback
            warnings.filterwarnings("ignore", category=PWMSoftwareFallback)
            self.trigger_pin = trigger_pin
            self.echo_pin = echo_pin
            self.sensor = DistanceSensor(echo=self.echo_pin, trigger=self.trigger_pin, max_distance=3)
        except ImportError:
            raise RuntimeError("gpiozero library not available")

    def get_distance(self):
        # Wrapped in try/except to prevent background-thread NoneType crash
        try:
            distance_cm = self.sensor.distance * 100
            return round(float(distance_cm), 1)
        except Exception:
            return -1

    def close(self):
        if hasattr(self, 'sensor'):
            try:
                self.sensor.close()
            except Exception:
                pass


class lgpiod_ultrasonic:
    def __init__(self, trigger_pin=27, echo_pin=22):
        try:
            import lgpio
            self.lgpio = lgpio
            self.trigger_pin = trigger_pin
            self.echo_pin = echo_pin
            try:
                self.chip = lgpio.gpiochip_open(0)
            except:
                self.chip = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(self.chip, self.trigger_pin)
            lgpio.gpio_claim_input(self.chip, self.echo_pin)
        except ImportError:
            raise RuntimeError("lgpio library not available")

    def get_distance(self):
        try:
            lgpio = self.lgpio
            lgpio.gpio_write(self.chip, self.trigger_pin, 0)
            time.sleep(0.05)
            lgpio.gpio_write(self.chip, self.trigger_pin, 1)
            time.sleep(0.00001)
            lgpio.gpio_write(self.chip, self.trigger_pin, 0)
            timeout = time.time() + 1.0
            start_time = time.time()
            while lgpio.gpio_read(self.chip, self.echo_pin) == 0:
                start_time = time.time()
                if start_time > timeout:
                    return -1
            stop_time = time.time()
            while lgpio.gpio_read(self.chip, self.echo_pin) == 1:
                stop_time = time.time()
                if stop_time > timeout:
                    return -1
            duration = stop_time - start_time
            distance = (duration * 34300) / 2
            return round(float(distance), 1)
        except Exception:
            return -1

    def close(self):
        if hasattr(self, 'chip') and self.chip is not None:
            try:
                self.lgpio.gpiochip_close(self.chip)
                self.chip = None
            except Exception:
                pass


class Ultrasonic:
    def __init__(self, trigger_pin=27, echo_pin=22):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.param_manager = ParameterManager()
        self.pi_version = self.param_manager.get_pi_version()
        self.sensor = None

        if self.pi_version == 2:
            print("Using lgpiod_ultrasonic")
            self.sensor = lgpiod_ultrasonic(trigger_pin, echo_pin)
        else:
            print("Using gpiozero_ultrasonic")
            self.sensor = gpiozero_ultrasonic(trigger_pin, echo_pin)

    def get_distance(self):
        if self.sensor is None:
            return -1
        return self.sensor.get_distance()

    def close(self):
        if self.sensor is not None:
            self.sensor.close()
            self.sensor = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# =============================================================================
# motor.py — tankMotor
# =============================================================================

class tankMotor:
    def __init__(self):
        from gpiozero import Motor
        self.left_motor  = Motor(24, 23)
        self.right_motor = Motor(5, 6)

    def duty_range(self, duty1, duty2):
        if duty1 > 4095:
            duty1 = 4095
        elif duty1 < -4095:
            duty1 = -4095
        if duty2 > 4095:
            duty2 = 4095
        elif duty2 < -4095:
            duty2 = -4095
        return duty1, duty2

    def left_Wheel(self, duty):
        if duty > 0:
            self.left_motor.forward(duty / 4096)
        elif duty < 0:
            self.left_motor.backward(-duty / 4096)
        else:
            self.left_motor.stop()

    def right_Wheel(self, duty):
        if duty > 0:
            self.right_motor.forward(duty / 4096)
        elif duty < 0:
            self.right_motor.backward(-duty / 4096)
        else:
            self.right_motor.stop()

    def setMotorModel(self, duty1, duty2):
        duty1, duty2 = self.duty_range(duty1, duty2)
        self.left_Wheel(duty1)
        self.right_Wheel(duty2)

    def close(self):
        self.left_motor.close()
        self.right_motor.close()


# =============================================================================
# servo.py — PigpioServo, GpiozeroServo, HardwareServo, Servo
# =============================================================================

class PigpioServo:
    def __init__(self):
        import pigpio
        self.channel1 = 7
        self.channel2 = 8
        self.channel3 = 25
        self.PwmServo = pigpio.pi()
        self.PwmServo.set_mode(self.channel1, pigpio.OUTPUT)
        self.PwmServo.set_mode(self.channel2, pigpio.OUTPUT)
        self.PwmServo.set_mode(self.channel3, pigpio.OUTPUT)
        self.PwmServo.set_PWM_frequency(self.channel1, 50)
        self.PwmServo.set_PWM_frequency(self.channel2, 50)
        self.PwmServo.set_PWM_frequency(self.channel3, 50)
        self.PwmServo.set_PWM_range(self.channel1, 4000)
        self.PwmServo.set_PWM_range(self.channel2, 4000)
        self.PwmServo.set_PWM_range(self.channel3, 4000)

    def setServoPwm(self, channel, angle):
        if channel == '0':
            self.PwmServo.set_PWM_dutycycle(self.channel1, 80 + (400 / 180) * angle)
        elif channel == '1':
            self.PwmServo.set_PWM_dutycycle(self.channel2, 80 + (400 / 180) * angle)
        elif channel == '2':
            self.PwmServo.set_PWM_dutycycle(self.channel3, 80 + (400 / 180) * angle)


class GpiozeroServo:
    def __init__(self):
        from gpiozero import AngularServo
        self.channel1 = 7
        self.channel2 = 8
        self.channel3 = 25
        self.myCorrection = 0.0
        self.maxPW = (2.5 + self.myCorrection) / 1000
        self.minPW = (0.5 - self.myCorrection) / 1000
        self.servo1 = AngularServo(self.channel1, initial_angle=0, min_angle=0, max_angle=180,
                                   min_pulse_width=self.minPW, max_pulse_width=self.maxPW)
        self.servo2 = AngularServo(self.channel2, initial_angle=0, min_angle=0, max_angle=180,
                                   min_pulse_width=self.minPW, max_pulse_width=self.maxPW)
        self.servo3 = AngularServo(self.channel3, initial_angle=0, min_angle=0, max_angle=180,
                                   min_pulse_width=self.minPW, max_pulse_width=self.maxPW)

    def setServoPwm(self, channel, angle):
        if channel == '0':
            self.servo1.angle = angle
        elif channel == '1':
            self.servo2.angle = angle
        elif channel == '2':
            self.servo3.angle = angle


class HardwareServo:
    def __init__(self, pcb_version):
        from rpi_hardware_pwm import HardwarePWM
        self.pcb_version = pcb_version
        self.pwm_gpio12 = None
        self.pwm_gpio13 = None
        if self.pcb_version == 1:
            self.pwm_gpio12 = HardwarePWM(pwm_channel=0, hz=50, chip=0)
            self.pwm_gpio13 = HardwarePWM(pwm_channel=1, hz=50, chip=0)
        elif self.pcb_version == 2:
            self.pwm_gpio12 = HardwarePWM(pwm_channel=0, hz=50, chip=0)
            self.pwm_gpio13 = HardwarePWM(pwm_channel=1, hz=50, chip=0)
        self.pwm_gpio12.start(0)
        self.pwm_gpio13.start(0)

    def setServoStop(self, channel):
        if channel == '0':
            self.pwm_gpio12.stop()
        elif channel == '1':
            self.pwm_gpio13.stop()

    def setServoFrequency(self, channel, freq):
        if channel == '0':
            self.pwm_gpio12.change_frequency(freq)
        elif channel == '1':
            self.pwm_gpio13.change_frequency(freq)

    def setServoDuty(self, channel, duty):
        if channel == '0':
            self.pwm_gpio12.change_duty_cycle(duty)
        elif channel == '1':
            self.pwm_gpio13.change_duty_cycle(duty)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def setServoPwm(self, channel, angle):
        if channel == '0':
            duty = self.map(angle, 0, 180, 2.5, 12.5)
            self.setServoDuty(channel, duty)
        elif channel == '1':
            duty = self.map(angle, 0, 180, 2.5, 12.5)
            self.setServoDuty(channel, duty)


class Servo:
    def __init__(self):
        self.param = ParameterManager()
        self.pcb_version = self.param.get_pcb_version()
        self.pi_version = self.param.get_raspberry_pi_version()

        if self.pcb_version == 1 and self.pi_version == 1:
            self.pwm = GpiozeroServo()
        elif self.pcb_version == 1 and self.pi_version == 2:
            self.pwm = GpiozeroServo()
        elif self.pcb_version == 2 and self.pi_version == 1:
            self.pwm = HardwareServo(1)
        elif self.pcb_version == 2 and self.pi_version == 2:
            self.pwm = HardwareServo(2)
        self.pwm.setServoPwm('0', 90)
        self.pwm.setServoPwm('1', 140)

    def angle_range(self, channel, init_angle):
        if channel == '0':
            if init_angle < 90:
                init_angle = 90
            elif init_angle > 150:
                init_angle = 150
        elif channel == '1':
            if init_angle < 90:
                init_angle = 90
            elif init_angle > 150:
                init_angle = 150
        elif channel == '2':
            if init_angle < 0:
                init_angle = 0
            elif init_angle > 180:
                init_angle = 180
        return init_angle

    def setServoAngle(self, channel, angle):
        angle = self.angle_range(str(channel), int(angle))
        self.pwm.setServoPwm(str(channel), int(angle))

    def setServoStop(self):
        if self.pcb_version == 2:
            self.pwm.setServoStop('0')
            self.pwm.setServoStop('1')


# =============================================================================
# Red_Ball_Detection.py — RedBallDetector
# =============================================================================

class RedBallDetector:
    """
    Uses OpenCV to detect a red ball from the camera feed.
    detect() returns (detected, center_x, radius)
      detected  — True if a red ball is found
      center_x  — horizontal position in the 320-wide frame (0=left, 320=right)
      radius    — size of the ball in pixels (larger = closer)
    """

    FRAME_WIDTH  = 320
    FRAME_HEIGHT = 240

    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
        if not self.cap.isOpened():
            print("Warning: camera not available, red ball detection disabled.")

    def detect(self):
        if not self.cap.isOpened():
            return False, 0, 0

        ret, frame = self.cap.read()
        if not ret:
            return False, 0, 0

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red wraps around the hue spectrum — need two ranges
        lower_red1 = np.array([0,   120,  70])
        upper_red1 = np.array([10,  255, 255])
        lower_red2 = np.array([170, 120,  70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask  = cv2.bitwise_or(mask1, mask2)

        # Remove noise
        mask = cv2.erode(mask,  None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest)
            M = cv2.moments(largest)
            if radius > 10 and M["m00"] > 0:
                center_x = int(M["m10"] / M["m00"])
                return True, center_x, radius

        return False, 0, 0

    def close(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()


# =============================================================================
# car.py — Car  (motor speeds halved from original + red ball + obstacle fix)
# =============================================================================

class Car:
    def __init__(self):
        self.servo      = None
        self.sonic      = None
        self.motor      = None
        self.infrared   = None
        self.detector   = None
        self.start()

    def start(self):
        self.clamp_mode        = 0
        self.infrared_run_stop = False
        if self.servo    is None: self.servo    = Servo()
        if self.sonic    is None: self.sonic    = Ultrasonic()
        if self.motor    is None: self.motor    = tankMotor()
        if self.infrared is None: self.infrared = Infrared()
        if self.detector is None: self.detector = RedBallDetector()

    def close(self):
        self.clamp_mode = 0
        self.servo.setServoStop()
        self.sonic.close()
        self.motor.close()
        self.infrared.close()
        self.detector.close()
        self.servo    = None
        self.sonic    = None
        self.motor    = None
        self.infrared = None
        self.detector = None

    # ── Ultrasonic-only obstacle avoidance ───────────────────────────────────

    def mode_ultrasonic(self):
        distance = self.sonic.get_distance()
        if distance != 0:
            if distance < 45:
                self.motor.setMotorModel(-750, -750)   # was -1500, -1500
                time.sleep(0.4)
                self.motor.setMotorModel(-750, 750)    # was -1500,  1500
                time.sleep(0.2)
            else:
                self.motor.setMotorModel(750, 750)     # was  1500,  1500
        time.sleep(0.2)

    # ── Line follow + obstacle avoidance + red ball pickup ───────────────────

    def mode_infrared(self):
        distance       = self.sonic.get_distance()
        infrared_value = self.infrared.read_all_infrared()

        # 1. Red ball detection — highest priority
        ball_detected, center_x, radius = self.detector.detect()

        if ball_detected:
            print(f"Red ball detected: center_x={center_x}, radius={radius:.1f}, distance={distance}")

            # Close enough to pick up
            if distance > 5.0 and distance <= 12.0:
                self.motor.setMotorModel(0, 0)
                self.set_mode_clamp(1)
                while self.get_mode_clamp() == 1 and self.infrared_run_stop == False:
                    self.mode_clamp()
                if self.infrared_run_stop:
                    self.motor.setMotorModel(0, 0)
                    return
                self.motor.setMotorModel(-750, 750)    # was -1500,  1500
                time.sleep(1.5)
                self.motor.setMotorModel(0, 0)
                self.set_mode_clamp(2)
                while self.get_mode_clamp() == 2 and self.infrared_run_stop == False:
                    self.mode_clamp()
                if self.infrared_run_stop:
                    self.motor.setMotorModel(0, 0)
                    return
                self.motor.setMotorModel(750, -750)    # was  1500, -1500
                time.sleep(1.4)

            else:
                # Steer toward the ball using center_x
                # Frame is 320 px wide: left zone <120, centre 120-200, right zone >200
                if center_x < 120:
                    self.motor.setMotorModel(-750, 1250)   # turn left toward ball
                elif center_x > 200:
                    self.motor.setMotorModel(1250, -750)   # turn right toward ball
                else:
                    self.motor.setMotorModel(600, 600)     # drive straight at ball
            return

        # 2. Obstacle avoidance (12-45 cm, no ball) — medium priority
        if distance > 12.0 and distance < 45.0:
            self.motor.setMotorModel(-750, -750)       # was -1500, -1500
            time.sleep(0.4)
            self.motor.setMotorModel(-750, 750)        # was -1500,  1500
            time.sleep(0.2)
            return

        # 3. Line following — base behaviour
        if infrared_value == 2:
            self.motor.setMotorModel(600, 600)         # was  1200,  1200
        elif infrared_value == 4:
            self.motor.setMotorModel(-750, 1250)       # was -1500,  2500
        elif infrared_value == 6:
            self.motor.setMotorModel(-1000, 2000)      # was -2000,  4000
        elif infrared_value == 1:
            self.motor.setMotorModel(1250, -750)       # was  2500, -1500
        elif infrared_value == 3:
            self.motor.setMotorModel(2000, -1000)      # was  4000, -2000
        elif infrared_value == 7:
            self.motor.setMotorModel(0, 0)

    # ── Clamp operations ─────────────────────────────────────────────────────

    def mode_clamp_up(self):
        if self.clamp_mode == 1:
            distance = self.sonic.get_distance()
            print("car_mode_clamp_up distance:", distance)
            if distance <= 5:
                self.motor.setMotorModel(-600, -600)   # was -1200, -1200
            elif distance > 5 and distance < 7.5:
                self.motor.setMotorModel(-400, -400)   # was  -800,  -800
            elif distance >= 7.5 and distance <= 7.7:
                self.motor.setMotorModel(0, 0)
                for i in range(140, 90, -1):
                    self.servo.setServoAngle('1', i)
                    time.sleep(0.01)
                for i in range(90, 130, 1):
                    self.servo.setServoAngle('0', i)
                    time.sleep(0.01)
                for i in range(90, 140, 1):
                    self.servo.setServoAngle('1', i)
                    time.sleep(0.01)
                self.clamp_mode = 0
            elif distance > 7.7 and distance < 11:
                self.motor.setMotorModel(400, 400)     # was   800,   800
            elif distance >= 11:
                self.motor.setMotorModel(600, 600)     # was  1200,  1200
            time.sleep(0.1)                            # increased from 0.05 to reduce sensor hammering

    def mode_clamp_down(self):
        if self.clamp_mode == 2:
            self.motor.setMotorModel(0, 0)
            for i in range(140, 90, -1):
                self.servo.setServoAngle('1', i)
                time.sleep(0.01)
            for i in range(130, 90, -1):
                self.servo.setServoAngle('0', i)
                time.sleep(0.01)
            for i in range(90, 140, 1):
                self.servo.setServoAngle('1', i)
                time.sleep(0.01)
            self.clamp_mode = 0

    def mode_clamp_stop(self):
        self.motor.setMotorModel(0, 0)

    def set_mode_clamp(self, mode=0):
        self.clamp_mode = mode

    def get_mode_clamp(self):
        return self.clamp_mode

    def mode_clamp(self, mode=None):
        if mode is not None:
            self.clamp_mode = mode
        if self.clamp_mode == 1:
            self.mode_clamp_up()
        elif self.clamp_mode == 2:
            self.mode_clamp_down()
        elif self.clamp_mode == 0:
            self.mode_clamp_stop()


# =============================================================================
# Entry point
# =============================================================================

if __name__ == '__main__':
    car = Car()
    print("Robot started. Press Ctrl+C to stop.")
    try:
        while True:
            car.mode_infrared()
    except KeyboardInterrupt:
        car.close()
        print("\nRobot stopped.")
