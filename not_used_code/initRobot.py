from board import SCL, SDA
import busio
import time
import yaml
import math
import torch
import atexit
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685
import csv
import numpy as np

class Rover():
    def __init__(self, config_filename):
        self.doLogging = True
        if self.doLogging:
            f = open('/home/xavier/isaac_rover_physical/exomy/scripts/utils/csv/MotorCommands.csv', 'w')
            self.writer = csv.writer(f)
        self.i2c_bus = busio.I2C(SCL, SDA)
        self.module = PCA9685(self.i2c_bus)
        self.module.frequency = 50
        self.cycle = 1.0/self.module.frequency * 1000.0  # ms
        #config_filename = '/root/exomy_ws/src/exomy/config/exomy.yaml'
        self.steeringPins = self.get_steering_motor_pins(config_filename)
        self.steeringNeutrals = self.get_steering_pwm_neutral_values(config_filename)

        self.drivePins = self.get_driving_pins(config_filename)
        self.driveNeutral = self.get_drive_pwm_neutral(config_filename)
        self.drivePWMrange = self.get_drive_pwm_range(config_filename)
        self.steerPWMrange = self.get_steer_pwm_range(config_filename)
        self.startup_sequence()
        atexit.register(self.exit_handler)
        

    def setMotorsFromKinematics(self, steering_angles, motor_velocities):
        steeringAnglesMotors = {'pin_steer_fl': 'steer_pwm_neutral_fl', 'pin_steer_fr': 'steer_pwm_neutral_fr', 
            'pin_steer_cl': 'steer_pwm_neutral_cl', 'pin_steer_cr': 'steer_pwm_neutral_cr', 
                'pin_steer_rl': 'steer_pwm_neutral_rl', 'pin_steer_rr': 'steer_pwm_neutral_rr'}
        steeringValues = {'pin_steer_fl': 0, 'pin_steer_fr': 1, 'pin_steer_cl': 2, 'pin_steer_cr': 3, 'pin_steer_rl': 4, 'pin_steer_rr': 5}
        driveValues = {'pin_drive_fl': 0, 'pin_drive_fr': 1, 'pin_drive_cl': 2, 'pin_drive_cr': 3, 'pin_drive_rl': 4, 'pin_drive_rr': 5}
        if self.doLogging:
            self.writer.writerow(np.array([steering_angles.tolist(), motor_velocities.tolist()]).flatten())
        #steering_angles = steering_angles.cpu().detach().numpy()
        #motor_velocities = motor_velocities.cpu().detach().numpy()
        #print(len(steering_angles))

        for pin_name, pin_num in self.steeringPins.items():
            #print((pin_name, pin_num))
            self.module.channels[self.steeringPins[pin_name]].duty_cycle  = self.remap(
                steering_angles[steeringValues[pin_name]], -math.pi/2, math.pi/2, 
                    self.steeringNeutrals[steeringAnglesMotors[pin_name]] - (self.steerPWMrange/2), 
                        self.steeringNeutrals[steeringAnglesMotors[pin_name]] + (self.steerPWMrange/2))
        #print(int(steeringAnglesMotors[0][0]))
        for pin_name, pin_num in self.drivePins.items():
            self.module.channels[self.drivePins[pin_name]].duty_cycle  = self.remap(
                motor_velocities[driveValues[pin_name]], (-math.pi * 1.66), (math.pi * 1.66), 
                    self.driveNeutral - self.drivePWMrange/2, self.driveNeutral + self.drivePWMrange/2)

        
    def get_steering_motor_pins(self, config_filename):
        steering_motor_pins = {}
        with open(config_filename, 'r') as file:
            param_dict = yaml.safe_load(file)

        for param_key, param_value in param_dict.items():
            if('pin_steer_' in str(param_key)):
                steering_motor_pins[param_key] = param_value
        return steering_motor_pins

    def get_steering_pwm_neutral_values(self, config_filename):
        steering_pwm_neutral_values = {}
        with open(config_filename, 'r') as file:
            param_dict = yaml.safe_load(file)

        for param_key, param_value in param_dict.items():
            if('steer_pwm_neutral_' in str(param_key)):
                steering_pwm_neutral_values[param_key] = param_value
        return steering_pwm_neutral_values

    def get_driving_pins(self, config_filename):
        drive_motor_pins = {}
        with open(config_filename, 'r') as file:
            param_dict = yaml.safe_load(file)

        for key, value in param_dict.items():
            if('pin_drive_' in str(key)):
                drive_motor_pins[key] = value
        return drive_motor_pins

    def get_drive_pwm_neutral(self, config_filename):
        
        with open(config_filename, 'r') as file:
            param_dict = yaml.safe_load(file)

        for key, value in param_dict.items():
            if('drive_pwm_neutral' in str(key)):
                return value           

    def get_drive_pwm_range(self, config_filename):
        
        with open(config_filename, 'r') as file:
            param_dict = yaml.safe_load(file)

        for key, value in param_dict.items():
            if('drive_pwm_range' in str(key)):
                return value            

    def get_steer_pwm_range(self, config_filename):
        
        with open(config_filename, 'r') as file:
            param_dict = yaml.safe_load(file)

        for key, value in param_dict.items():
            if('steer_pwm_range' in str(key)):
                return value            
    def startup_sequence(self):
        


        # for i in range(180):
        #     on_time = 0.5 + (((2.25-0.5)/180) * i)
        #     duty_cycle = int((on_time/self.cycle)*65536)
        #     for pin_name, pin_num in self.steeringPins.items():
        #         self.module.channels[self.steeringPins[pin_name]].duty_cycle = duty_cycle
        #     time.sleep(0.001)

        # for i in range(180):
        #     on_time = 2.25 - ((2.25-0.5)/180) * i
        #     duty_cycle = int((on_time/self.cycle)*65536)
        #     for pin_name, pin_num in self.steeringPins.items():
        #         self.module.channels[self.steeringPins[pin_name]].duty_cycle = duty_cycle
        #     time.sleep(0.001)

        self.module.channels[self.steeringPins['pin_steer_fl']].duty_cycle = self.steeringNeutrals['steer_pwm_neutral_fl']
        self.module.channels[self.steeringPins['pin_steer_fr']].duty_cycle = self.steeringNeutrals['steer_pwm_neutral_fr']
        self.module.channels[self.steeringPins['pin_steer_cl']].duty_cycle = self.steeringNeutrals['steer_pwm_neutral_cl']
        self.module.channels[self.steeringPins['pin_steer_cr']].duty_cycle = self.steeringNeutrals['steer_pwm_neutral_cr']
        self.module.channels[self.steeringPins['pin_steer_rl']].duty_cycle = self.steeringNeutrals['steer_pwm_neutral_rl']
        self.module.channels[self.steeringPins['pin_steer_rr']].duty_cycle = self.steeringNeutrals['steer_pwm_neutral_rr']

        # self.module.channels[self.drivePins['pin_drive_fl']].duty_cycle = self.driveNeutral + 100
        # self.module.channels[self.drivePins['pin_drive_fr']].duty_cycle = self.driveNeutral - 100
        # self.module.channels[self.drivePins['pin_drive_cl']].duty_cycle = self.driveNeutral + 100
        # self.module.channels[self.drivePins['pin_drive_cr']].duty_cycle = self.driveNeutral - 100
        # self.module.channels[self.drivePins['pin_drive_rl']].duty_cycle = self.driveNeutral + 100
        # self.module.channels[self.drivePins['pin_drive_rr']].duty_cycle = self.driveNeutral - 100
        # time.sleep(0.5)

        # self.module.channels[self.drivePins['pin_drive_fl']].duty_cycle = self.driveNeutral - 100
        # self.module.channels[self.drivePins['pin_drive_fr']].duty_cycle = self.driveNeutral + 100
        # self.module.channels[self.drivePins['pin_drive_cl']].duty_cycle = self.driveNeutral - 100
        # self.module.channels[self.drivePins['pin_drive_cr']].duty_cycle = self.driveNeutral + 100
        # self.module.channels[self.drivePins['pin_drive_rl']].duty_cycle = self.driveNeutral - 100
        # self.module.channels[self.drivePins['pin_drive_rr']].duty_cycle = self.driveNeutral + 100
        time.sleep(0.5)
        for pin_name, pin_num in self.drivePins.items():
            self.module.channels[self.drivePins[pin_name]].duty_cycle = self.driveNeutral

    def remap(self, x, oMin, oMax, nMin, nMax ):

        #range check
        if oMin == oMax:
            print("Warning: Zero input range")
            return None

        if nMin == nMax:
            print("Warning: Zero output range")
            return None

        #check reversed input range
        reverseInput = False
        oldMin = min( oMin, oMax )
        oldMax = max( oMin, oMax )
        if not oldMin == oMin:
            reverseInput = True

        #check reversed output range
        reverseOutput = False   
        newMin = min( nMin, nMax )
        newMax = max( nMin, nMax )
        if not newMin == nMin :
            reverseOutput = True

        portion = (x-oldMin)*(newMax-newMin)/(oldMax-oldMin)
        if reverseInput:
            portion = (oldMax-x)*(newMax-newMin)/(oldMax-oldMin)

        result = portion + newMin
        if reverseOutput:
            result = newMax - portion

        return int(result)

        
    def exit_handler(self):
        
        self.module.channels[self.drivePins['pin_drive_fl']].duty_cycle = 0
        self.module.channels[self.drivePins['pin_drive_fr']].duty_cycle = 0
        self.module.channels[self.drivePins['pin_drive_cl']].duty_cycle = 0
        self.module.channels[self.drivePins['pin_drive_cr']].duty_cycle = 0
        self.module.channels[self.drivePins['pin_drive_rl']].duty_cycle = 0
        self.module.channels[self.drivePins['pin_drive_rr']].duty_cycle = 0
        self.module.channels[self.steeringPins['pin_steer_fl']].duty_cycle = 0
        self.module.channels[self.steeringPins['pin_steer_fr']].duty_cycle = 0
        self.module.channels[self.steeringPins['pin_steer_cl']].duty_cycle = 0
        self.module.channels[self.steeringPins['pin_steer_cr']].duty_cycle = 0
        self.module.channels[self.steeringPins['pin_steer_rl']].duty_cycle = 0
        self.module.channels[self.steeringPins['pin_steer_rr']].duty_cycle = 0

