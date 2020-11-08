# GY88
Driver for the GY88 IMU sensor

## Information
This code is a simple driver intended to be used in a flight computer. The code uses a complimentary fusion filter to estimate the attitude of the chip using the MPU6050 sensor and the magnetometer MMC5883. In order for the code to work you have to have the MPU6050, MMC5883 and BMP085 repositories.

The algorithm itself to estimate the attitude is accurate and fast and relies mainly on the accelerometer data for short time frames and is corrected over time by the magnetometer and gyro input.

## Example usage:
```
>>import time
>>import GY88

>>  # Fire up the IMU
    imu = gy88()
    
    # Calibrate sensors
    imu.mpu.calibrate()
    imu.mmc.calibrate()

    # Initial attitude, magnetic reference and timestep
    attitude, dt = [0.0, 0.0, 0.0], 0.01
    while True:
        start = time.perf_counter()
        attitude = imu.attitude_est(attitude, dt)
        
        # Print attitude and convert heading range from (-180, 180) to (0, 360)
        print("pitch: ", int(attitude[0]),"roll: ", int(attitude[1]),"yaw: ",int((360 + attitude[2]) % 360))
        time.sleep(0.01)
        dt = time.perf_counter() - start
        
pitch:  0 roll:  0 yaw:  356
pitch:  0 roll:  1 yaw:  352
pitch:  0 roll:  2 yaw:  351
pitch:  0 roll:  3 yaw:  351
pitch:  0 roll:  3 yaw:  351
pitch:  0 roll:  3 yaw:  351
pitch:  0 roll:  5 yaw:  352
pitch:  0 roll:  5 yaw:  352
pitch:  0 roll:  5 yaw:  353
pitch:  0 roll:  5 yaw:  354...
```
