from time import sleep
from gpiozero import PWMOutputDevice as PWM
from gpiozero import DigitalOutputDevice as DOD
from gyro import Gyro

class Motor:

    def __init__(self, in1, in2, pwm):
        self.in1 = DOD(in1)
        self.in2 = DOD(in2)
        self.pwm = PWM(pwm)
        self.pwm.value = 0


    def forward(self, pwm):
        self.in1.on()
        self.in2.off()
        self.pwm.value = pwm


    def backward(self, pwm):
        self.in1.off()
        self.in2.on()
        self.pwm.value = pwm

    def stop(self):
        self.pwm.value = 0

class motorControl():

    def __init__(self, AIN1 = 26, AIN2 = 20, BIN1 = 16, BIN2 = 19, PWMA = 12, PWMB = 13):
        self.motor_left = Motor(AIN1, AIN2, PWMA)
        self.motor_right = Motor(BIN1, BIN2, PWMB)
        self.gyro = Gyro()

    def set_speed_direction(self, speed, direction):
        if direction >= 0:
            pwm_left = round(1 * speed)
            pwm_right = round((1 - direction) * speed)
        else:
            pwm_left = round((1 + direction) * speed)
            pwm_right = round(1 * speed)
        if speed >= 0:
            self.motor_left.forward(pwm_left)
            self.motor_right.forward(pwm_right)
        else:
            self.motor_left.backward(pwm_left)
            self.motor_right.backward(pwm_right)

    def drive(self, left_speed, right_speed, direction = True):
        if direction:
            self.motor_left.forward(left_speed)
            self.motor_right.forward(right_speed)
        else:
            self.motor_left.backward(left_speed)
            self.motor_right.backward(right_speed)

    def straight(self, speed, direction = True, targetDistance = None):
        if not targetDistance:
            self.drive(speed, speed, direction)
        else:
            # TODO implement targetDistance using encoder
            return

    def spin_right(self, speed, targetAngle = None):
        if not targetAngle:
            self.motor_left.backward(speed)
            self.motor_right.forward(speed)
        else:
            self.gyro.reset()
            while True:
                angle = self.gyro.get_z_cumulative()
                # print(angle)
                self.motor_left.backward(speed)
                self.motor_right.forward(speed)
                if angle < targetAngle:
                    break
                sleep(0.1)
            self.stop()

    def spin_left(self, speed, targetAngle = None):
        if not targetAngle:
            self.motor_left.forward(speed)
            self.motor_right.backward(speed)
        else:
            self.gyro.reset()
            while True:
                angle = self.gyro.get_z_cumulative()
                # print(angle)
                self.motor_left.forward(speed)
                self.motor_right.backward(speed)
                if angle > targetAngle:
                    break
                sleep(0.1)
            self.stop()
        

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()


if __name__ == "__main__":

    car_motor_control = motorControl()
    print("motorControl.py is running")
    car_motor_control.drive(0.2, 0.2)
