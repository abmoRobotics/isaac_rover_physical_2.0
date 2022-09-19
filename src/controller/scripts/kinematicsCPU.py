import numpy as np
import math


def kinematicsCPU(lin_vel_x, ang_vel):
        '''
        Converts the steering command [angle of joystick] to angles for the different motors
        '''
        FL, FR, CL, CR, RL, RR = range(0, 6)


        steering_angles = [0.0]*6
        motor_speeds = [0.0]*6
        if ang_vel != 0:
            radius = (abs(lin_vel_x)/abs(ang_vel))*100

        # Distance from center og the rover to the top (centimeters):
        y_top = 19.5 # check if it's correct
        # Distance from center of the rover to the side (centimeters):
        x_side = 15 # check if it's correct

        """
        Steering angles calculation 
        """
        # If the angular velociy is 0, the angles for the wheel are set to 0
        if ang_vel == 0: 
            steering_angles[FL] = 0.0
            steering_angles[FR] = 0.0
            steering_angles[CL] = 0.0
            steering_angles[CR] = 0.0
            steering_angles[RL] = 0.0
            steering_angles[RR] = 0.0
        # If the turning point is within the chassis of the robot, turn on the spot:
        elif radius <= x_side : 
            steering_angles[FL] = math.atan2(y_top,x_side)
            steering_angles[FR] = -math.atan2(y_top,x_side)
            steering_angles[CL] = 0.0
            steering_angles[CR] = 0.0
            steering_angles[RL] = -math.atan2(y_top,x_side)
            steering_angles[RR] = math.atan2(y_top,x_side)
        # Steering angles if turning anticlockwise moving forward or clockwise moving backwards
        elif (ang_vel > 0 and np.sign(lin_vel_x) > 0) or (ang_vel < 0 and np.sign(lin_vel_x) < 0):
            steering_angles[FL] = -math.atan2(y_top,(radius-x_side))
            steering_angles[FR] = -math.atan2(y_top,(radius+x_side))
            steering_angles[CL] = 0.0
            steering_angles[CR] = 0.0
            steering_angles[RL] = math.atan2(y_top,(radius-x_side))
            steering_angles[RR] = math.atan2(y_top,(radius+x_side))
        # Steering angles if turning clockwise moving forward or anticlockwise moving backwards
        elif (ang_vel < 0 and np.sign(lin_vel_x) > 0) or (ang_vel > 0 and np.sign(lin_vel_x) < 0):
            steering_angles[FL] = math.atan2(y_top,(radius+x_side))
            steering_angles[FR] = math.atan2(y_top,(radius-x_side))
            steering_angles[CL] = 0.0
            steering_angles[CR] = 0.0
            steering_angles[RL] = -math.atan2(y_top,(radius+x_side))
            steering_angles[RR] = -math.atan2(y_top,(radius-x_side))

        """
        Motor speeds calculation
        """
        # Speed moving forward/backward = linear velocity 
        if ang_vel == 0: 
            motor_speeds[FL] = float(-lin_vel_x)
            motor_speeds[FR] = float(lin_vel_x)
            motor_speeds[CL] = float(-lin_vel_x)
            motor_speeds[CR] = float(lin_vel_x)
            motor_speeds[RL] = float(-lin_vel_x)
            motor_speeds[RR] = float(lin_vel_x)
        # Speed turning in place (anticlockwise), velocity of corner wheels = angular velocity 
        elif radius <= x_side and ang_vel > 0: 
            frontLeft = math.sqrt((y_top*y_top)+(x_side*x_side))*abs(ang_vel)
            centerLeft = x_side*abs(ang_vel)
            relation = centerLeft/frontLeft # relation between corner wheel and center wheel velocity (center wheels slower)
            motor_speeds[FL] = ang_vel
            motor_speeds[FR] = ang_vel
            motor_speeds[CL] = ang_vel*relation
            motor_speeds[CR] = ang_vel*relation
            motor_speeds[RL] = ang_vel
            motor_speeds[RR] = ang_vel
        # Speed turning in place (clockwise), velocity of corner wheels = angular velocity 
        elif radius <= x_side and ang_vel < 0: 
            frontLeft = math.sqrt((y_top*y_top)+(x_side*x_side))*abs(ang_vel)
            centerLeft = x_side*abs(ang_vel)
            relation = centerLeft/frontLeft # relation between corner wheel and center wheel velocity (center wheels slower)
            motor_speeds[FL] = ang_vel
            motor_speeds[FR] = ang_vel
            motor_speeds[CL] = ang_vel*relation
            motor_speeds[CR] = ang_vel*relation
            motor_speeds[RL] = ang_vel
            motor_speeds[RR] = ang_vel
        # Speed turning anticlockwise moving forward/backward, velocity of frontRight wheel = linear velocity 
        elif ang_vel > 0:
            frontLeft = (math.sqrt((y_top*y_top)+((radius-x_side)*(radius-x_side)))*abs(ang_vel))*np.sign(lin_vel_x)
            frontRight = (math.sqrt((y_top*y_top)+((radius+x_side)*(radius+x_side)))*abs(ang_vel))*np.sign(lin_vel_x)
            frontRelation = frontLeft/frontRight # relation of speed between the front wheels (frontLeft is slower)
            centerLeft = ((radius-x_side)*abs(ang_vel))*np.sign(lin_vel_x)
            centerRight = ((radius+x_side)*abs(ang_vel))*np.sign(lin_vel_x)
            centerRelation = centerLeft/centerRight # relation of speed between the center wheels (centerLeft is slower)
            frontCenterRelation = centerRight/frontRight # relation between center and front wheels (center is slower)
            motor_speeds[FL] = -lin_vel_x*frontRelation
            motor_speeds[FR] = lin_vel_x
            motor_speeds[CL] = -lin_vel_x*frontCenterRelation*centerRelation
            motor_speeds[CR] = lin_vel_x*frontCenterRelation
            motor_speeds[RL] = -lin_vel_x*frontRelation
            motor_speeds[RR] = lin_vel_x
        # Speed turning clockwise moving forward/backward, velocity of frontLeft wheel = linear velocity
        elif ang_vel < 0:
            frontLeft = (math.sqrt((y_top*y_top)+((radius+x_side)*(radius+x_side)))*abs(ang_vel))*np.sign(lin_vel_x)
            frontRight = (math.sqrt((y_top*y_top)+((radius-x_side)*(radius-x_side)))*abs(ang_vel))*np.sign(lin_vel_x)
            frontRelation = frontRight/frontLeft # relation of speed between the front wheels (frontRight is slower)
            centerLeft = ((radius+x_side)*abs(ang_vel))*np.sign(lin_vel_x)
            centerRight = ((radius-x_side)*abs(ang_vel))*np.sign(lin_vel_x)
            centerRelation = centerRight/centerLeft # relation of speed between the center wheels (centerRight is slower)
            frontCenterRelation = centerLeft/frontLeft # relation between center and front wheels (center is slower)
            motor_speeds[FL] = -lin_vel_x
            motor_speeds[FR] = lin_vel_x*frontRelation
            motor_speeds[CL] = -lin_vel_x*frontCenterRelation
            motor_speeds[CR] = lin_vel_x*frontCenterRelation*centerRelation
            motor_speeds[RL] = -lin_vel_x
            motor_speeds[RR] = lin_vel_x*frontRelation
        #motor_speeds = motor_speeds * 0.3
        #new_list = [item / 3 for item in motor_speeds]
        return steering_angles, motor_speeds