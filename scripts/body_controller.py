import math
sim = None
simVision = None
bot_body = None
right_motor = None
left_motor = None
vision_sensor = None
dt = None
goal_reached = False
stable_time = 0
current_state = None  
target_state = None
error = None
control = 0
previous_position = None  
previous_tilt = None      
yaw_rate = 0
max_speed = 2            
max_yaw_rate = 15        
goal_position = [1, 1]
obstacle_detected = False
avoidance_offset = 0  

def sysCall_init():
    global sim, simVision, bot_body, right_motor, left_motor, vision_sensor, backUntilTime
    global dt, current_state, target_state, error, previous_position, previous_tilt, nose
    sim = require('sim')
    simVision = require('simVision')
    bot_body = sim.getObject('/body')
    right_motor = sim.getObject('/body/right_joint')
    left_motor = sim.getObject('/body/left_joint')
    vision_sensor = sim.getObject('/sensor')
    dt = sim.getSimulationTimeStep()
    transformation_matrix = sim.getObjectMatrix(bot_body, -1)
    nose = sim.getObject("../Proximity_sensor") 
    rotation_part = [transformation_matrix[:3],
                     transformation_matrix[4:7],
                     transformation_matrix[8:11]]
    initial_position = [transformation_matrix[3],
                        transformation_matrix[7],
                        transformation_matrix[11]]
    initial_tilt = math.atan2(rotation_part[2][1], rotation_part[2][2])
    current_state = [initial_position[1], 0.0, initial_tilt, 0.0]  
    target_state = [0.0, 0.0, 0.0, 0.0]  
    error = [0.0, 0.0, 0.0, 0.0]
    previous_position = initial_position
    previous_tilt = initial_tilt
    robotCollection = sim.createCollection(0)
    sim.addItemToCollection(robotCollection, sim.handle_tree, bot_body, 0)
    backUntilTime=-1
    
def sysCall_vision(inData):
    global obstacle_detected, avoidance_offset
    simVision.sensorImgToWorkImg(inData['handle'])
    simVision.edgeDetectionOnWorkImg(inData['handle'], 0.2)
    simVision.workImgToSensorImg(inData['handle'])
        
def sysCall_actuation():
    global current_state, control, previous_position, previous_tilt, stable_time, nose, backUntilTime
    transformation_matrix = sim.getObjectMatrix(bot_body, -1)
    rotation_part = [transformation_matrix[:3],
                     transformation_matrix[4:7],
                     transformation_matrix[8:11]]
    tilt_angle = math.atan2(rotation_part[2][1], rotation_part[2][2])
    position = [transformation_matrix[3],
                transformation_matrix[7],
                transformation_matrix[11]]
    position_delta = [position[i] - previous_position[i] for i in range(3)]
    previous_position = position  
    local_delta_y = (rotation_part[0][1] * position_delta[0] +
                     rotation_part[1][1] * position_delta[1] +
                     rotation_part[2][1] * position_delta[2])
    linear_velocity_y = local_delta_y / dt
    angular_velocity_x = (tilt_angle - previous_tilt) / dt
    previous_tilt = tilt_angle  
    current_state[0] = position[1]
    current_state[1] = linear_velocity_y
    current_state[2] = tilt_angle
    current_state[3] = angular_velocity_x
    error[0] = target_state[0] - current_state[0]
    error[1] = target_state[1] - current_state[1]
    error[2] = target_state[2] - current_state[2]
    error[3] = target_state[3] - current_state[3]
    
    k = [-3, -500.04317888, 145.86856503, 6.39583786]
    control = (k[0] * error[0] +
               k[1] * error[1] +
               k[2] * error[2] +
               k[3] * error[3]) 
    
    err_turn = goal_position[0] - position[1]
    
    error_x = goal_position[0]
    error_y = goal_position[1]
    distance_to_goal = math.sqrt(error_x**2 + error_y**2)
    
    result, *_ = sim.readProximitySensor(nose) 
    if result > 0:
        backUntilTime = sim.getSimulationTime() + 10
    else:
        backUntilTime = -1
    
    if abs(err_turn) < 0.05 and backUntilTime < sim.getSimulationTime():
        right_motor_speed = control 
        left_motor_speed = control 
        
    elif abs(err_turn) > 0.05 and backUntilTime > sim.getSimulationTime():
        right_motor_speed = control + 25
        left_motor_speed = control - 25
    
    elif abs(err_turn) > 0.05 and backUntilTime < sim.getSimulationTime():
        target_angle = math.atan2(error_y, error_x)
        current_angle = math.atan2(rotation_part[0][1], rotation_part[0][0])
        angle_error = target_angle - current_angle
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        yaw_adjustment = math.copysign(abs(angle_error) * 10, angle_error)
        movement_adjustment = math.copysign(25, distance_to_goal)
        right_motor_speed = control + yaw_adjustment - movement_adjustment
        left_motor_speed = control - yaw_adjustment - movement_adjustment

    sim.setJointTargetVelocity(right_motor, right_motor_speed)
    sim.setJointTargetVelocity(left_motor, left_motor_speed)
        
def sysCall_sensing():
    global yaw_rate
    yaw_rate = 0
