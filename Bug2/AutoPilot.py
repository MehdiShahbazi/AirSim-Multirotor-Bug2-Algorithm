import os
import time
import airsim
import numpy as np

from Env import DroneEnv


class AutoPilot:
    def __init__(self, params):
        
        # Extract general params
        self.save_path              = params['save_path']
        self.selected_env           = params['selected_env']
        self.weather_params         = params['weather_params']
        self.plot_path_planning     = params['plot_path_planning']
        self.save_plot_list         = params['save_plot_list']        
 
        # Make saving directory
        os.makedirs(self.save_path, exist_ok=True)
                                
        # Define Env
        self.env = DroneEnv(self.weather_params)
        self.obstacles = ['SM_Truck_Box', 'Sedan_SkelMesh', 'SM_SUV', 'SM_SportsCar']
                      
        
    def angle_to_target(self, position, quaternion):
        angle_to_target = self.env.calculate_angle(position, quaternion, for_Mline=True)
        return angle_to_target

    
    def dist_to_M_line(self, quad_position):
        quad_to_init_vector = quad_position - self.env.initial_position
        
        # Vector from quad position to M-line (Projection vector of the quad's position onto the M-line.)
        quad_to_mline = np.dot(quad_to_init_vector, self.m_line_unit_vector) * self.m_line_unit_vector

        # Distance to M-line
        dist_to_mline = np.linalg.norm(quad_position - (self.env.initial_position + quad_to_mline))
        return dist_to_mline
    
    
    def Bug2(self, ):        
        # Initialize quad connection and define variables
        prev_bug2_state = 'start'
        bug2_state = 'align_robot_heading'

        hit_point = [] # x, y
        leave_point = [] # x, y
        
        theta_threshold = 4 # degrees threshold to goal (M-line)
        acceptable_range = 2  # acceptable degree range
        in_range_threshold = 2 # number of times the angle is in correct range before change state
        sensor_noise = 0.82 # distance sensor of airsim has a gaussian noise and this value helps in reduce te noise impact on decision
        distance_to_M_line_threshold = 0.3
        last_pass_distance = np.inf
        initial_wall_distance = -np.inf
        turn_start_timer = np.inf
        max_wait_time = 5
        
        turn_timer_checker = False
        obstacle_in_list = False
        use_pos_x = False
        measure_wall_distance = False
        front_collision_checker = False
        measure_object_scale = False
        occupied_right_space, occupied_left_space = False, False
        forward_before_wall_distance = False
        front_collision = False
        rotate_right = False
        initial_front_sensor = False
        collision_move = False
        first_angle_target = True

        front_collision_forward_count = 0 # steps before front sensor value is used for checking distance from left or right side of quad
        front_collision_false_alarm = 0
        in_range_count = 0 # steps that angle is stable and correct for moving
        backward_count = 0
        forward_count = 0
        
        # Calculate the initial M-line
        m_line_vector = self.env.goal - self.env.initial_position
        self.m_line_unit_vector = m_line_vector / np.linalg.norm(m_line_vector)
        
        # Only one restart is sufficient
        option = {
            'plot_path_planning' : self.plot_path_planning,
            'save_plot_list'     : self.save_plot_list
            }
        
        # Reset from start point - New Beginning
        self.env.reset(option)
        self.env.client.simGetCollisionInfo().has_collided
        start = time.time()
            
        # Loop over episodes to gather samples
        while True:
            solved = False
    
            sensor_data = self.env.all_distance_sensors() # Retrieve all sensors data
            position, quaternion = self.env.current_position() # Retrieve current position & quaternion (orientation) data
            is_rate, angle_value = True, 0

            #~~ align_robot_heading
            if bug2_state == 'align_robot_heading':
                """Aligns the heading angle of the quad to the target goal position."""
                angle_target = self.angle_to_target(position, quaternion)
                if theta_threshold > min(angle_target, 360-angle_target):
                    prev_bug2_state = bug2_state
                    bug2_state = 'move_to_goal'
                else: # robot heading is not aligned, thus rotate by yaw
                    if angle_target <= 180:
                        action = 4 # rotate right
                    else:
                        action = 5 # rotate left
                        
                    
            #~~ move_to_goal
            if bug2_state == 'move_to_goal':
                """Moves the quad towards the goal position."""
                # check if collided
                collision_info = self.env.client.simGetCollisionInfo()
                if collision_info.has_collided and collision_info.object_name != '(null)':
                    hit_point.append(np.array([collision_info.impact_point.x_val, 
                                               collision_info.impact_point.y_val]))
                    object_id = collision_info.object_name

                    # check whether the obstacles collided is in the list or not
                    for item in self.obstacles:
                        if object_id.startswith(item):
                            obstacle_in_list = True # False esh kojast
                            if obstacle_in_list:
                                break
                    
                    if obstacle_in_list:
                        object_quaternion = self.env.client.simGetObjectPose(object_id).orientation
                        euler = airsim.to_eularian_angles(object_quaternion) # Convert object quaternion to Euler angles   
                        object_angle = np.degrees(euler[2]) % 180  # Extract object angle from Euler angles and ensure it's in the range of 0 to 180 degrees
                    else: # if obstacle not in the list, then quad should calculate another angle
                        object_angle = self.env.north_angle                    

                    prev_bug2_state = bug2_state
                    bug2_state = 'move_backward'
                else:
                    action = 0 # move forward   
                    
                    
            #~~ move_backward
            elif bug2_state == 'move_backward':
                """Moves the quad backward after collision to get distance from the collided object."""
                action = 1 # move backward
                backward_count += 1
                if backward_count == 3:
                    prev_bug2_state = bug2_state
                    bug2_state = 'turn'
                    backward_count = 0
            

            #~~ turn      
            elif bug2_state == 'turn':
                """Turns the quad to navigate around obstacles after collision."""
                is_rate, angle_value = False, object_angle
                
                if not turn_timer_checker:
                    turn_start_timer = time.time()
                    turn_timer_checker = True

                # when quad was in moving forward state and sensed that front sensor has lower range, then it means quad has rotated in wrong direction. thus:
                if prev_bug2_state == 'move_forward':
                    if first_angle_target:
                        # front_sensor_distance = sensor_data['0']
                        euler = airsim.to_eularian_angles(quaternion)
                        quad_first_angle = round(np.degrees(euler[2]) % 180) # quad initial angle when state is move_forward
                        angle_to_target = self.angle_to_target(position, quaternion) % 180
                        angle_difference = quad_first_angle - angle_to_target
                        first_angle_target = False
                        front_collision = True

                    # quad current angle
                    euler = airsim.to_eularian_angles(quaternion)
                    quad_angle = round(np.degrees(euler[2]) % 180)  
                    
                    if angle_difference <= 0: # difference of target angle and quad angle
                        action = 5 # rotate left
                    else:
                        action = 4 # rotate right
                    
                    if action == 4:
                        angle_value = quad_first_angle + 90
                    else:
                        angle_value = quad_first_angle - 90
                    
                    opposite_front_collision_action = 4 if action == 5 else 5
                    angle_dist = np.linalg.norm((angle_value % 180) - quad_angle)
                    is_in_range_count = angle_dist < acceptable_range
                    
                else:    
                    if round(object_angle) >= 90:
                        action = 4 # rotate right
                    else:
                        action = 5 # rotate left
                    
                    # for reverse movement on the other side of the obstacle
                    if action == 4:
                        opposite_action = 5
                    elif action == 5:
                        opposite_action = 4
                        
                    # check whether turn has been fully done or not
                    euler = airsim.to_eularian_angles(quaternion)
                    quad_angle = round(np.degrees(euler[2]) % 180)
                    # Calculate the valid bound of the acceptable range
                    if quad_angle >= 170: # because degree more than 180 becomes 0 but 180 is close to 0 (pervent ocillation between 180 and 0)
                        valid_bound = (180 - quad_angle)
                    else:
                        valid_bound = quad_angle                             
                    # is_in_range_count = np.linalg.norm(object_angle - valid_bound) < acceptable_range
                    # print(object_angle, valid_bound, quad_angle, is_in_range_count)
                    is_in_range_count = np.linalg.norm(object_angle - quad_angle) < acceptable_range
                    # print(object_angle, quad_angle, is_in_range_count)
                if is_in_range_count:
                    in_range_count += 1
                    if in_range_count == in_range_threshold or time.time() - turn_start_timer > max_wait_time:
                        prev_bug2_state = bug2_state
                        bug2_state = 'move_forward'
                        in_range_count = 0
                        first_angle_target = True
                        turn_start_timer = np.inf
                        turn_timer_checker = False
                        in_range_count = 0
                else:
                    in_range_count = 0


            #~~ move_forward                
            elif bug2_state == 'move_forward':
                """Moves the quad forward along the obstacle after collision and turning from collision."""             
                if not initial_front_sensor:
                    # if front sensor has low distance it means quad has not turned to correct direction
                    front_sensor = self.env.client.getDistanceSensorData('Distance_0')
                    distance_condition = front_sensor.max_distance - front_sensor.min_distance - sensor_noise
                    initial_front_sensor = True

                if front_sensor.distance > distance_condition and front_sensor.distance > initial_wall_distance - front_sensor.min_distance - sensor_noise:
                    action = 0 # move forward
                    forward_count += 1
                    # if quad is collided change state (# Fail-safe)
                    if self.env.client.simGetCollisionInfo().has_collided or forward_count == 32:
                        prev_bug2_state = bug2_state
                        bug2_state = 'align_robot_heading'
                            
   
                    if forward_before_wall_distance and prev_bug2_state == 'corner_turn':  
                        front_collision_forward_count += 1
                        front_collision_checker = True
                        if front_collision_forward_count == 7:
                            # calculate the first measure 
                            forward_before_wall_distance = False
                            collision_move = True
                            
                    elif prev_bug2_state == 'corner_turn' and not front_collision_checker:
                        # check whether quad is back on M-Line or not
                        distance_to_M_line = self.dist_to_M_line(position)
                        is_on_M_line = distance_to_M_line < distance_to_M_line_threshold
                        if is_on_M_line:
                            prev_bug2_state = bug2_state
                            bug2_state = 'align_robot_heading'
                            leave_point.append(position)

                    elif not measure_wall_distance:
                        # check which side is free
                        if sensor_data['90'] + sensor_noise < sensor_data['-90']: # right space is occupied
                            occupied_right_space = True
                            initial_wall_distance = sensor_data['90'] + sensor_noise
                        else:
                            occupied_left_space = True
                            initial_wall_distance = sensor_data['-90'] + sensor_noise
                        measure_wall_distance = True
                            
                    if measure_wall_distance: # time to check distance from sides                                                
                        # check whether quad is in corner and should turn or not
                        if initial_wall_distance < sensor_data['90'] and occupied_right_space: # thus the right spaced is being freed and time to turn
                            rotate_right = True
                            prev_bug2_state = bug2_state
                            if not obstacle_in_list or collision_move:
                                bug2_state = 'align_robot_heading'
                            else:
                                bug2_state = 'corner_turn'
    
                        elif initial_wall_distance < sensor_data['-90'] and occupied_left_space:
                            prev_bug2_state = bug2_state
                            if not obstacle_in_list or collision_move:
                                bug2_state = 'align_robot_heading'
                            else:
                                bug2_state = 'corner_turn'
                                                                                                  
                else:
                    # print(f'{front_sensor.distance:.1f}, {distance_condition:.1f}, {initial_wall_distance:.1f}')
                    prev_bug2_state = bug2_state
                    bug2_state = 'turn'
                    forward_before_wall_distance = True
                    action = 1 # move one step backward
             
                if bug2_state != 'move_forward':
                    initial_wall_distance = -np.inf
                    forward_count = 0
                    front_collision_forward_count = 0
                    measure_wall_distance = False
                    occupied_right_space, occupied_left_space = False, False
                    front_collision_checker = False
                    obstacle_in_list = False
                    collision_move = False
                    initial_front_sensor = False


            #~~ corner_turn                
            elif bug2_state == 'corner_turn':       
                """When encountered a corner executes turning to continue rounding the obstacle."""
                is_rate = False
                        
                if not turn_timer_checker:
                    turn_start_timer = time.time()
                    turn_timer_checker = True
                    
                if front_collision and sensor_data['0'] and sensor_data['-90'] and sensor_data['90'] > 4.5:
                    front_collision_false_alarm += 1
                    
                    if front_collision_false_alarm > 5:
                        front_collision = False
                        front_collision_false_alarm = 0
                        
                        
                if prev_bug2_state == 'move_forward' and front_collision:
                    angle_value = quad_first_angle
                    action = opposite_front_collision_action

                elif prev_bug2_state == 'move_forward':
                    if rotate_right:
                        orthogonal_value = 90 # rotate right
                        action = 4 # rotate right
                    else:
                        orthogonal_value = -90 # rotate left
                        action = 5 # rotate left
                    angle_value = round(object_angle + orthogonal_value) % 180
                elif prev_bug2_state == 'forward_after_turn':
                    angle_value = valid_bound + 180
                    action = opposite_action

                # check whether turn has been fully done or not
                euler = airsim.to_eularian_angles(quaternion)
                quad_angle = round(np.degrees(euler[2]) % 180)
                # print()
                # print(f'{front_collision}, quad_angle: {quad_angle:.2f}, angle_value: {angle_value:.2f}, object_angle: {object_angle:.2f}')
                # check whether quad is rotated and ready to move forward
                is_in_range_count = np.linalg.norm(quad_angle - (angle_value % 180)) < acceptable_range
                if is_in_range_count:
                    in_range_count += 1
                    if in_range_count == in_range_threshold:

                        if (prev_bug2_state == 'move_forward' and front_collision) or time.time() - turn_start_timer > max_wait_time:
                            prev_bug2_state = bug2_state
                            bug2_state = 'move_forward'
                            front_collision = False
                            opposite_front_collision_action = None

                        elif prev_bug2_state == 'move_forward' or time.time() - turn_start_timer > max_wait_time:
                            prev_bug2_state = bug2_state
                            bug2_state = 'forward_after_turn'
                        elif prev_bug2_state == 'forward_after_turn' or time.time() - turn_start_timer > max_wait_time:
                            prev_bug2_state = bug2_state
                            bug2_state = 'move_forward'
                            
                        if bug2_state != 'corner_turn':
                            turn_start_timer = np.inf
                            turn_timer_checker = False
                            in_range_count = 0
                            
                else:
                    in_range_count = 0
                
                              
            #~~ forward_after_turn   
            elif bug2_state == 'forward_after_turn':
                """Proceeds forward after a turn to resume the navigation."""
                if not measure_object_scale:
                    object_width = self.env.client.simGetObjectScale(object_id).y_val
                    euler = airsim.to_eularian_angles(quaternion)
                    angle_of_quad = round(np.degrees(euler[2]) % 180)
                    # print(abs(quad_angle - self.env.north_angle))
                    if abs(angle_of_quad - self.env.north_angle) >= 65:
                        quad_position = position[0] # only x_val
                        use_pos_x = True
                    else:
                        quad_position = position[1] # only y_val
                    
                    if object_width == 0.75:
                        prefixed_constant = 2.5
                    elif object_width == 1.0 or object_width == 1.25:
                        prefixed_constant = 3.5
                    elif object_width == 1.5:
                        prefixed_constant = 4.3
                    elif object_width == 1.75:
                        prefixed_constant = 4.6

                    pass_position = quad_position + object_width + prefixed_constant
                    measure_object_scale = True
                
                action = 0 # move forward
                if use_pos_x:
                    distance_to_pass_position = np.linalg.norm(position[0] - pass_position)
                else:
                    distance_to_pass_position = np.linalg.norm(position[1] - pass_position)
                passed_condition = distance_to_pass_position < 0.2

                if distance_to_pass_position > last_pass_distance: # fail-safe
                    passed_condition = True
                    
                if passed_condition:
                    prev_bug2_state = bug2_state
                    bug2_state = 'corner_turn'
                    
                    rotate_right = False
                    use_pos_x = False
                    measure_object_scale = False
                    last_pass_distance = np.inf
                    
                else:
                    last_pass_distance = distance_to_pass_position # fail-safe. saves last distance to pass position for tracking if condition didn't work, terminate it

                                       
            # print(f'prev: {prev_bug2_state} | curr: {bug2_state} | act: {action}')
            # print(f'distance to M-line: {self.dist_to_M_line(self.env.initial_position, position, self.env.goal):.2f}')
            
            solved = self.env.step(action, is_rate, angle_value)
            time.sleep(0.25) # delay for correct movement
            
            if solved:
                # Write log of training
                elapsed_time = abs(round(time.time() - start, 2))
                minutes = int(elapsed_time // 60)
                seconds = int(round(elapsed_time % 60, 2))
                result = ("Navigation Done. "
                          f"Time: {minutes}:{seconds}\n")
                print(result)
            if solved:
                if self.save_plot_list:
                    self.env.save_plot_lines(self.save_path + '/path_plot_points' + '.pbz2')
                print('Run Finished.')
                break
            
                      
if __name__ == "__main__":
    # Selent and define parameters:
    selected_env = 'ModularBuildingSet'
    weather_options = {'None' : 0, 'Rain' : 0, 'Snow' : 0, 'MapleLeaf' : 0, 'Dust' : 0, 'Fog' : 0}
    
    # select the weather
    selected_weathers = ['Snow', 'Fog'] # if None wanted add 'None' or replace whole with ['None']
    weather_values = [20, 0.05] # % In percentage    
    
    # Don't change the following
    weather_params = {}
    if 'None' not in selected_weathers:
        for i, j in enumerate(selected_weathers):
            weather_params[j] = weather_values[i]
    else:
        weather_params = None
        
        
    params = {
        'save_path'             : f'./Saved/{selected_env}',
        'selected_env'          : selected_env,
        'weather_params'        : weather_params,
        'plot_path_planning'    : False,
        'save_plot_list'        : True,
        }
    
    instance = AutoPilot(params)
    # Run Bug2 Algorithm
    instance.Bug2()