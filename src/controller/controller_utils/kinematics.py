#!/usr/bin/env python3
# kinematics.py
import torch
import numpy

def Ackermann(lin_vel, ang_vel, device):
    """
    Returns the values in rad/sec = moter velocities after gearbox and rad = stering angles.
    Returns the values ( [ FL, FR, CL, CR, RL, RR ],[ FL_ang, FR_ang, 0, 0, RL_ang, RR_ang ] )
    FL      = Front velocity left
    FR      = Front velocity right
    CL      = Center velocity left
    CR      = Center velocity right
    RL      = Rear velocity left
    RR      = Rear velocity right
    FL_ang  = Front angle left 
    FR_ang  = Front angle right
    RL_ang  = Rear angle left
    RR_ang  = Rear angle right
    """
    # type: (Tensor, Tensor, str) -> Tuple[Tensor, Tensor]
    # All measurements in Meters!

    scale = 1

    num_robots = 1
    wheel_diameter = 0.2 * scale
    lin_vel = torch.tensor([lin_vel], device=device)
    ang_vel = torch.tensor([ang_vel], device=device)

    # Locations of the wheels, with respect to center(between middle wheels) (Y is forward, X is right)
    wheel_FL = torch.unsqueeze(torch.transpose(torch.tensor(  [[-0.385],[0.438]],  device=device).repeat(1,num_robots), 0, 1),0)
    wheel_FR = torch.unsqueeze(torch.transpose(torch.tensor(  [[0.385],[0.438]],   device=device).repeat(1,num_robots), 0, 1),0)
    wheel_ML = torch.unsqueeze(torch.transpose(torch.tensor(  [[-0.447],[0.0]],    device=device).repeat(1,num_robots), 0, 1),0)
    wheel_MR = torch.unsqueeze(torch.transpose(torch.tensor(  [[0.447],[0.0]],     device=device).repeat(1,num_robots), 0, 1),0)
    wheel_RL = torch.unsqueeze(torch.transpose(torch.tensor(  [[-0.385],[-0.411]], device=device).repeat(1,num_robots), 0, 1),0)
    wheel_RR = torch.unsqueeze(torch.transpose(torch.tensor(  [[0.385],[-0.411]],  device=device).repeat(1,num_robots), 0, 1),0)
    
    # Wheel locations, collected in a single variable
    wheel_locations = torch.cat((wheel_FL, wheel_FR, wheel_ML, wheel_MR, wheel_RL, wheel_RR), 0) * scale
    
    # The distance at which the rover should switch to turn on the spot mode.
    bound = 0.45 * scale

    # Turning point
    P = torch.unsqueeze(lin_vel/ang_vel, 0)
    P = torch.copysign(P, -ang_vel)
    zeros = torch.zeros_like(P)
    P = torch.transpose(torch.cat((P,zeros), 0), 0, 1) # Add a zero component in the y-direction.
    P[:,0] = torch.squeeze(torch.where(torch.abs(P[:,0]) > bound, P[:,0], zeros)) # If turning point is between wheels, turn on the spot.
    lin_vel = torch.where(P[:,0] != 0, lin_vel, zeros) # If turning on the spot, set lin_vel = 0.

    # Calculate distance to turning point
    P = P.repeat((6,1,1))
    dist = torch.transpose((P - wheel_locations).pow(2).sum(2).sqrt(), 0, 1)

    # Motors on the left should turn opposite direction
    motor_side = torch.transpose(torch.tensor([[-1.0],[1.0],[-1.0],[1.0],[-1.0],[1.0]], device=device).repeat((1, num_robots)), 0, 1)
    
    # When not turning on the spot, wheel velocity is actually determined by the linear direction
    wheel_linear = torch.transpose(torch.copysign(ang_vel, lin_vel).repeat((6,1)), 0, 1)
    # When turning on the spot, wheel velocity is determined by motor side.
    wheel_turning = torch.transpose(ang_vel.repeat((6,1)), 0, 1) * motor_side
    ang_velocities = torch.where(torch.transpose(lin_vel.repeat((6,1)), 0, 1) != 0, wheel_linear, wheel_turning)
    
    # The velocity is determined by the disance from the wheel to the turning point, and the angular velocity the wheel should travel with
    motor_velocities = dist * ang_velocities

    # If the turning point is more than 1000 meters away, just go straight.
    motor_velocities = torch.where(dist > 1000, torch.transpose(lin_vel.repeat((6,1)), 0, 1), motor_velocities)

    # Convert linear velocity above ground to rad/s
    motor_velocities = (motor_velocities/wheel_diameter)*scale
    
    steering_angles = torch.transpose(torch.where(torch.abs(wheel_locations[:,:,0]) > torch.abs(P[:,:,0]), torch.atan2(wheel_locations[:,:,1], wheel_locations[:,:,0] - P[:,:,0]), torch.atan2(wheel_locations[:,:,1], wheel_locations[:,:,0] - P[:,:,0])), 0, 1)
    steering_angles = torch.where(steering_angles < -3.1415/2, steering_angles + 3.1415, steering_angles)
    steering_angles = torch.where(steering_angles > 3.1415/2, steering_angles - 3.1415, steering_angles)

    # Sets the wheels straigt when at stand-still
    steering_angles = torch.where(torch.abs(motor_velocities) < 0.001, torch.zeros_like(steering_angles), steering_angles)

    steering_angles = steering_angles.squeeze().detach().numpy()
    motor_velocities = motor_velocities.squeeze().detach().numpy()
    
    return steering_angles, motor_velocities
