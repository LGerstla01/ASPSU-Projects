#!interpreter
# -*- coding: utf-8 -*-

"""
Demo of the gyro only mode of a Bosch BNO055 development board.
"""

# ######### Built-in/Generic imports ##########
from typing import Tuple, Type
import adafruit_bno055
import board
import time

__author__ = 'Nico Hessenthaler'
__copyright__ = 'Copyright 2023, HHN - Autonomous Systems: Perception and Situation Understanding'
__credits__ = ['-']
__license__ = '-'
__version__ = '1.0.0'
__maintainer__ = 'Nico Hessenthaler'
__email__ = 'nico.hessenthaler@gmail.com'
__status__ = 'Finished'


# ######### Source code ##########

def initialize_mpu_calibration(sensor: Type[adafruit_bno055.BNO055_I2C],
                               gyro_calibration_offsets: Tuple,
                               accelerometer_calibration_offsets: Tuple,
                               magnetometer_calibration_offsets: Tuple,
                               accelerometer_calibration_radius: int,
                               magnetometer_calibration_radius: int) -> Type[adafruit_bno055.BNO055_I2C]:
    """
    Method that initializes the mpu by writing the calibration values to the corresponding
    registers. BNO055 doesn't have an EEPROM to store data. After power loss, calibration is
    lost. Since calibration can't be done after every startup of the PI, the sensor has to be 
    calibrated once and calibration data must be reused after each start up.

    Args:
        sensor (adafruit_bno055): Instance of BNO055 sensor class.
        gyro_calibration_offsets (Tuple): Gyro calibration offsets in 3 dimensions.
        accelerometer_calibration_offsets (Tuple): Accelerometer calibration data in 3 dimensions.
        magnetometer_calibration_offsets (Tuple): Magnetometer calibration data in 3 dimensions.
        accelerometer_calibration_radius (int): Accelerometer calibration radius.
        magnetometer_calibration_radius (int): Magnetometer calibration radius.

    Returns:
        sensor (adafruit_bno055): Calibrated instance of BNO055 sensor class.
    """

    # Config mode to allow writing to registers
    sensor.mode = adafruit_bno055.CONFIG_MODE

    # Write the calibration values
    sensor.offsets_gyroscope = gyro_calibration_offsets
    sensor.offsets_accelerometer = accelerometer_calibration_offsets
    sensor.offsets_magnetometer = magnetometer_calibration_offsets
    sensor.radius_accelerometer = accelerometer_calibration_radius
    sensor.radius_magnetometer = magnetometer_calibration_radius

    # Wait until configuration is done
    time.sleep(0.1)

    # Re-enable full GYRONLY_MODE none fusion mode
    sensor.mode = adafruit_bno055.GYRONLY_MODE
    
    # Wait until all data is valid
    time.sleep(0.5)

    return sensor


def read_raw_gyroscope(sensor: Type[adafruit_bno055.BNO055_I2C]) -> Tuple:
    """
    Method that reads the current values of raw gyroscope from the registers of the BNO055.

    Args:
        sensor (adafruit_bno055): Instance of BNO055 sensor class.

    Returns:
        raw_gyroscope (Tuple): Raw gyroscope in x, y, z dimension.
    """

    # Read current value from register
    raw_gyroscope = sensor.gyro

    return raw_gyroscope



# Main function
if __name__=="__main__": 

    # Init BNO055 board
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    # BNO055 calibration data
    gyro_calibration_offsets = (-2, 2, -1)
    accelerometer_calibration_offsets = (-4, 14, -22)
    magnetometer_calibration_offsets = (-548, 197, 461)
    accelerometer_calibration_radius = 1000
    magnetometer_calibration_radius = 873
    sensor = initialize_mpu_calibration(sensor, \
                                        gyro_calibration_offsets, \
                                        accelerometer_calibration_offsets,
                                        magnetometer_calibration_offsets, \
                                        accelerometer_calibration_radius, \
                                        magnetometer_calibration_radius)

    while(1):

        raw_gyro = read_raw_gyroscope(sensor)
        if raw_gyro[0] is not None:
            print(f"Roll rate:{round(raw_gyro[0], 2): >6} rad/s - Pitch rate:{round(raw_gyro[1], 2): >6} rad/s - Yaw rate:{round(raw_gyro[2], 2): >6} rad/s")
        time.sleep(0.05)

#EOF
