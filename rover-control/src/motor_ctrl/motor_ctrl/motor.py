import math

import RPi.GPIO as GPIO


Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

BACKWARD = -1
FORWARD = 1

LEFT = -1
RIGHT = 1

class Motor(object):
    def __init__(self, logger):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Motor_A_EN, GPIO.OUT)
        GPIO.setup(Motor_B_EN, GPIO.OUT)
        GPIO.setup(Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(Motor_B_Pin2, GPIO.OUT)

        self.stop()
        self.pwm_A = GPIO.PWM(Motor_A_EN, 1000)
        self.pwm_B = GPIO.PWM(Motor_B_EN, 1000)
        self.speed_left = 0
        self.speed_right = 0
        self.logger = logger

    def stop(self):
        self.__stopLeft()
        self.__stopRight()

    def __stopRight(self):
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN, GPIO.LOW)
        self.speed_right = 0

    def __stopLeft(self):
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN, GPIO.LOW)
        self.speed_left = 0

    def __motor_right(self, direction, speed):
        speed = max(0, min(speed, 100))
        if direction == FORWARD:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            self.pwm_B.start(0)
            self.pwm_B.ChangeDutyCycle(speed)
            self.speed_right = speed
        elif direction == BACKWARD:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            self.pwm_B.start(100)
            self.pwm_B.ChangeDutyCycle(speed)
            self.speed_right = -speed

    def __motor_left(self, direction, speed):
        speed = max(0, min(speed, 100))
        if direction == FORWARD:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            self.pwm_A.start(0)
            self.pwm_A.ChangeDutyCycle(speed)
            self.speed_left = speed
        elif direction == BACKWARD:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            self.pwm_A.start(100)
            self.pwm_A.ChangeDutyCycle(speed)
            self.speed_left = -speed

    def drive(self, speed, direction=FORWARD):
        self.__motor_left(direction, speed)
        self.__motor_right(direction, speed)

    def set_motor_speeds(self, speed_left, speed_right):
        self.__motor_left(math.copysign(1, speed_left), 100 * abs(speed_left))
        self.__motor_right(math.copysign(1, speed_right), 100 * abs(speed_right))

    def turn(self, amount):
        if amount > 0:
            self.__motor_left(BACKWARD, amount)
            self.__motor_right(FORWARD, amount)
        elif amount < 0:
            self.__motor_left(FORWARD, -amount)
            self.__motor_right(BACKWARD, -amount)

    def changeSpeedMult(self, side, amount):
        dL = 1 + amount if side == RIGHT else 1 - amount
        dR = 1 + amount if side == LEFT else 1 - amount

        newSpeed_left = min(abs(self.speed_left) * dL, 100)
        newSpeed_right = min(abs(self.speed_right) * dR, 100)

        self.__motor_left(math.copysign(1, self.speed_left), newSpeed_left)
        self.__motor_right(math.copysign(1, self.speed_right), newSpeed_right)

    def changeSpeedAdd(self, side, amount):
        steer_intensity = abs(0.01 * self.speed_left * self.speed_right) + 10
        dL = steer_intensity * amount if side == RIGHT else -steer_intensity * amount
        dR = -dL

        newSpeed_left = self.speed_left + dL
        newSpeed_right = self.speed_right + dR

        self.__motor_left(math.copysign(1, newSpeed_left), abs(newSpeed_left))
        self.__motor_right(math.copysign(1, newSpeed_right), abs(newSpeed_right))

    def destroy(self):
        self.stop()
        GPIO.cleanup()
