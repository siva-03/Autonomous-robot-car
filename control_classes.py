#!/home/odroid/.pyenv/shims/python
# license removed for brevity
import rospy
import time
import os

from car_control.utils import write_serial_byte_string


class PhysicalCar:
    def __init__(self, steering_channel=3, motor_channel=2):
        self.steering_channel = steering_channel
        self.motor_channel = motor_channel
        self.__steering = None
        self.__motor = None
        # finalize initialization by centering wheels and motor to 1500
        self.center()

    def center(self):
        # set the digital values
        self.__steering = 1500
        self.__motor = 1500
        # write the physical values to file
        write_serial_byte_string(channel=self.steering_channel, target=1500)
        write_serial_byte_string(channel=self.motor_channel, target=1500)

    @property
    def steering(self):
        return self.__steering

    @steering.setter
    def steering(self, value):
        try:
            sanitized_value = int(value)
            if 1000 <= sanitized_value <= 2000:
                self.__steering = value
                write_serial_byte_string(channel=self.steering_channel, target=sanitized_value)
            else:
                print("error: steering setter value not in maestro 1000-2000 range but was int: ", sanitized_value)
        except ValueError:
            print("error: steering setter value not an int")

    @steering.deleter
    def steering(self):
        print("deleting steering? dont think this should get called")
        self.__steering = 1500
        write_serial_byte_string(channel=self.steering_channel, target=1500)

    @property
    def motor(self):
        return self.__motor

    @motor.setter
    def motor(self, value):
        try:
            sanitized_value = int(value)
            if 1000 <= sanitized_value <= 2000:
                self.__motor = value
                write_serial_byte_string(channel=self.motor_channel, target=sanitized_value)
            else:
                print("error: motor setter received an int, but not in range 1000-2000: ", sanitized_value)
        except ValueError:
            print("error: motor setter value not an int")

    @motor.deleter
    def motor(self):
        print("deleting motor? dont think this should get called")
        self.__motor = 1500
        write_serial_byte_string(channel=self.motor_channel, target=1500)


class PIDController:
    def __init__(self, kp, ki, kd):
        self.integral = None
        self.last_error = None
        self.last_time = None
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def step(self, position, set_point=0.0):
        # Get current time
        current_time = time.time()

        # Calculate time elapsed since last iteration
        dt = current_time - self.last_time

        # Calculate error
        error = set_point - position

        # Calculate integral, accumulated error
        self.integral += error * dt

        # Calculate derivative of error
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0

        # Calculate output: kP + kI + kD
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Save last error and time
        self.last_error = error
        self.last_time = current_time

        # Return the raw output (we can transform it how we want, all the alg cares about is the error change)
        return output


class SensorData:
    def __init__(self):
        self.imu_data = None


class ImageData:
    def __init__(self):
        self.image_data = None
        self.width = None
        self.height = None
