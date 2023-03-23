import time
import board
import adafruit_fxas21002c
from motorControl import motorControl
from simple_pid import PID

i2c = board.I2C()
sensor = adafruit_fxas21002c.FXAS21002C(i2c)
motor = motorControl()


gyro_z_cumulative = 0

while True:
    motor.drive(1, 1)

exit()

while True:
    gyro_x, gyro_y, gyro_z = sensor.gyroscope
    gyro_z_degrees = gyro_z * 180 / 3.14159
    gyro_z_cumulative += gyro_z_degrees * 0.1
    print("Gyro Z: ", gyro_z_cumulative)
    if gyro_z_cumulative < 89:
        motor.spin_left(0.2)
    elif gyro_z_cumulative > 91:
        motor.spin_right(0.3)
    else:
        motor.stop()
        break
    
    time.sleep(0.1)

# i2c = board.I2C()
# sensor = adafruit_fxas21002c.FXAS21002C(i2c)
# motor = motorControl()

# pid = PID(1, 0.1, 0.05, setpoint=0)
# pid.output_limits = (-0.3, 0.3)
# gyro_z_cumulative = 0
# gyro_z_target = 90

# while True:
#     gyro_x, gyro_y, gyro_z = sensor.gyroscope
#     gyro_z_degrees = gyro_z * 180 / 3.14159
#     gyro_z_cumulative += gyro_z_degrees * 0.1
#     pid_input = gyro_z_cumulative
#     pid_output = pid(pid_input)
#     if pid_output >= 0:
#         motor.spin_left(pid_output)
#     else:
#         motor.spin_right(-pid_output)

#     print(pid_output, gyro_z_cumulative)
#     if abs(gyro_z_cumulative) >= 90:
#         motor.stop()
#         break
#     time.sleep(0.1)