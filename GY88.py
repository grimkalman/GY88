from MPU6050 import *
from MMC5883 import *
from BMP085 import *
from math import sqrt, atan2, sin, cos, pi
import time

# Contants
rad_to_deg = 180/pi
deg_to_rad = pi/180

class gy88():
    
    def __init__(self, bmp_address=0x77, mpu_address=0x68, mmc_address=0x30):
        # Initialize the sensors
        self.bmp = mmc5883(bmp_address)
        self.mpu = mpu6050(mpu_address)
        self.mmc = mmc5883(mmc_address)
        
    def attitude_est(self, attitude, dt):
        # Read sensor data
        ax, ay, az = self.mpu.get_accel()
        gx, gy, gz = self.mpu.get_gyro()
        mx, my, mz = self.mmc.get_mag()
        
        
        # Fuse sensor data using a complementary filter for pitch and roll data
        attitude[0] = 0.80*(attitude[0] + gx * dt) + 0.20*(atan2(ay, az)*rad_to_deg)
        attitude[1] = 0.80*(attitude[1] + gy * dt) + 0.20*(atan2(ax, az)*rad_to_deg)
        
        # Extract the magnetic field components in x and y direction (tilt compensated)
        bx = (mx*cos(attitude[1]*deg_to_rad) +
              my*sin(attitude[0]*deg_to_rad)*sin(attitude[1]*deg_to_rad) +
              mz*cos(attitude[0]*deg_to_rad)*sin(attitude[1]*deg_to_rad))
        by = my*cos(attitude[0]*deg_to_rad) - mz*sin(-attitude[0]*deg_to_rad)
        
        # Fuse sensor data using a complementary filter for yaw data
        attitude[2] = 0.80*(attitude[2] + gz * dt) + 0.20*atan2(bx, -by)*rad_to_deg
        
        return attitude
    

if __name__ == "__main__":
    # Fire up the IMU
    imu = gy88()
    
    # Calibrate sensors
    imu.mpu.calibrate()
#     imu.mmc.calibrate()

    # Initial attitude, magnetic reference and timestep
    attitude, dt = [0.0, 0.0, 0.0], 0.01
    while True:
        start = time.perf_counter()
        attitude = imu.attitude_est(attitude, dt)
        print("pitch: ", int(attitude[0]),"roll: ", int(attitude[1]),"yaw: ",int((360 + attitude[2]) % 360))
        time.sleep(0.01)
        dt = time.perf_counter() - start