import board
import adafruit_fxas21002c

class Gyro:
    def __init__(self):
        self.i2c = board.I2C()
        self.sensor = adafruit_fxas21002c.FXAS21002C(self.i2c)
        self.gyro_z_cumulative = 0

    def reset(self):
        self.gyro_z_cumulative = 0

    def get_z_speed(self):
        gyro_x, gyro_y, gyro_z = self.sensor.gyroscope
        gyro_z_degrees = gyro_z * 180 / 3.14159
        return gyro_z_degrees

    

    def get_z_cumulative(self):
        gyro_z_degrees = self.get_z_speed()
        self.gyro_z_cumulative += gyro_z_degrees * 0.1
        return self.gyro_z_cumulative

if __name__ == "__main__":
    gyro = Gyro()
    gyro.reset()
    while True:
        gyro_angle = gyro.get_z_cumulative()
        print("Gyro: ", gyro_angle)