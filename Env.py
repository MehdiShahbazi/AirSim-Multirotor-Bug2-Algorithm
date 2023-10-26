import bz2
import time
import pickle
import airsim
import numpy as np
# import gymnasium as gym


# class DroneEnv(gym.Env):
class DroneEnv():
    """Drone environment class using AirSim python API"""
    def __init__(self, weather_params):
        # General
        self.scaling_factor = 3.8 # action scaling factor
        self.plot_points = []
                
        # Parameters
        self.duration = 0.1 # seconds (Desired time to send this command for drivetrain)
        self.max_altitude = -1.15
                
        # Quad info variables
        self.client = airsim.MultirotorClient()
        self.setup() # Reset the drone on initialization
        time.sleep(0.25) # Delay for safe initialization
        
        # Weather Settings
        if weather_params is not None:
            self.client.simEnableWeather(True)
            for i, weather in enumerate(list(weather_params.keys())):
                if weather == 'Rain':
                    print('h')
                    self.client.simSetWeatherParameter(airsim.WeatherParameter.Rain, weather_params['Rain'])
                elif weather == 'Snow':
                    self.client.simSetWeatherParameter(airsim.WeatherParameter.Snow, weather_params['Snow'])
                elif weather == 'MapleLeaf':
                    self.client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, weather_params['MapleLeaf'])
                elif weather == 'Dust':
                    self.client.simSetWeatherParameter(airsim.WeatherParameter.Dust, weather_params['Dust'])
                elif weather == 'Fog':
                    self.client.simSetWeatherParameter(airsim.WeatherParameter.Fog, weather_params['Fog'])
        
        # Find the North angle
        north_angle = 90
        print('Finding the pre-defined north %d\u00B0 angle.' %north_angle)
        self.client.moveByVelocityZBodyFrameAsync(vx = 0, vy = 0, z  = -0.2, 
                duration = 3.2, yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=north_angle)).join()
        north_orientation = self.client.getMultirotorState().kinematics_estimated.orientation
        euler = airsim.to_eularian_angles(north_orientation) # Convert quaternion to Euler angles   
        self.north_angle = round(np.degrees(euler[2])) # Extract yaw angle from Euler angles
        self.setup() # Reset again
        time.sleep(0.1) # Delay for safe initialization
        
        # Goal setup
        object_id = "Target_Goal" # Target_Goal is a Empty and hidden actor placed in unreal world
        pose = self.client.simGetObjectPose(object_id).position # the object ID from unreal_engine World Outlier panel
        self.goal = np.array((pose.x_val, pose.y_val))
        
        # Quad initials
        position = self.client.getMultirotorState().kinematics_estimated.position
        self.initial_position = np.array([position.x_val, position.y_val])
        self.initial_distance = self.calculate_distance(self.initial_position, self.goal)
        self.quad_offset = (0, 0, 0) # [x, y, yaw]

      
    def setup(self):
        # Reset & takeoff
        self.client.reset()
        self.client.simPause(False)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
      
    def reset(self, option=None):
        # super().reset()
        
        self.plot_path_planning = option['plot_path_planning']
        self.save_plot_list = option['save_plot_list']
        
        # Reset & takeoff
        self.setup()
        self.client.moveByVelocityAsync(vx=0, vy=0, vz=-1.5, duration=0.4).join()
        
        # Get the initial distance
        KinematicsState = self.client.getMultirotorState().kinematics_estimated
        position = KinematicsState.position
 
        # Plot in AirSim
        if self.plot_path_planning or self.save_plot_list:
            self.append_plot_points(position)
            
        if self.plot_path_planning:
            self.plot_lines()


    def step(self, action, is_rate=True, angle_value=0):            
        # Interpret action
        self.interpret_action(action) # saves in self.quad_offset
        
        # Apply action
        if is_rate:
            self.client.moveByVelocityZBodyFrameAsync(
                vx = self.quad_offset[0], 
                vy = self.quad_offset[1], 
                z  = self.max_altitude, 
                duration = self.duration, 
                yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=self.quad_offset[2])
                ).join()
        else:
            self.client.moveByVelocityZBodyFrameAsync(
                vx = self.quad_offset[0], 
                vy = self.quad_offset[1], 
                z  = self.max_altitude, 
                duration = self.duration + 0.5, 
                yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=angle_value)
                ).join()
        
        # New Quad Kinematics
        KinematicsState = self.client.getMultirotorState().kinematics_estimated
        
        # Calculate reward
        position = KinematicsState.position
        position_array = np.array([position.x_val, position.y_val])
        new_quaternion = KinematicsState.orientation
        
        solved = self.compute_reward(position_array, new_quaternion)
        
        
        # Plot in AirSim
        if self.plot_path_planning or self.save_plot_list:
            self.append_plot_points(position)
            
        if self.plot_path_planning:
            self.plot_lines()
                            
        return solved
    
    
    def calculate_distance(self, position_array, target_point):
        """Get distance between current state and goal state"""
        distance = np.linalg.norm(position_array - target_point, ord=2) # norm 2
        return distance
    
    
    def calculate_angle(self, position_array, quaternion, for_Mline=False):  
        """Get angle between quad position to goal position"""       
        euler = airsim.to_eularian_angles(quaternion) # Convert quaternion to Euler angles   
        yaw_angle = np.degrees(euler[2]) # Extract yaw angle from Euler angles
        
        # Calculate direction vector from current position to goal position
        direction = self.goal - position_array
        # Calculate angle between direction vector and yaw angle
        heading_angle = np.degrees(np.arctan2(direction[1], direction[0])) - yaw_angle
        
        # Wrap the angle within 0 to 360 degrees
        theta = heading_angle % 360
        if for_Mline:
            return theta
        else:
            if theta > 180:
                theta = 360 - theta
            return theta
    

    def compute_reward(self, position_array, quaternion):
        solved = False
        self.client.simGetCollisionInfo().has_collided
        # according to distance
        current_distance = self.calculate_distance(position_array, self.goal)
        self.previous_distance = current_distance
        
        # if goal is close
        if current_distance < 3:
            solved = True
            print("======== + Solved + ========")

        return solved


    def interpret_action(self, action):
        """Interprete action"""
        if action == 0:
            self.quad_offset = (self.scaling_factor, 0, 0)   # Move Forward (increase X)
        elif action == 1:
            self.quad_offset = (-self.scaling_factor, 0, 0)  # Move Backward (decrease X)
        elif action == 2:
            self.quad_offset = (0, self.scaling_factor, 0)   # Move Right (increase Y)
        elif action == 3:
            self.quad_offset = (0, -self.scaling_factor, 0)  # Move Left (decrease Y)
        elif action == 4:
            self.quad_offset = (0, 0, self.scaling_factor*15)   # Rotate to Right (+)
        elif action == 5:
            self.quad_offset = (0, 0, -self.scaling_factor*15)  # Rotate to left (-)
            
    
    def dist_sensors(self):
        distance_sensorData = {
            '0'     : self.client.getDistanceSensorData('Distance_0').distance, 
            '-30'   : self.client.getDistanceSensorData('Distance_M30').distance,
            '30'    : self.client.getDistanceSensorData('Distance_30').distance,
            }
        return np.array(list(distance_sensorData.values()))
    
    
    def all_distance_sensors(self):
        distance_dict = {
            '0'     : self.client.getDistanceSensorData('Distance_0').distance, 
            '-30'   : self.client.getDistanceSensorData('Distance_M30').distance,
            '-90'   : self.client.getDistanceSensorData('Distance_M90').distance,
            '30'    : self.client.getDistanceSensorData('Distance_30').distance,
            '90'   : self.client.getDistanceSensorData('Distance_90').distance,
            }
        return distance_dict
    
    
    def current_position(self):
        position = self.client.getMultirotorState().kinematics_estimated.position
        orientation = self.client.getMultirotorState().kinematics_estimated.orientation
        position = np.array([position.x_val, position.y_val])
        return position, orientation
    
    
    def append_plot_points(self, position):
        self.plot_points.append(position)
    
    
    def plot_lines(self):
        self.client.simPlotLineList(points = self.plot_points, 
                                    color_rgba = [1.0, 0.0, 0.0, 1.0], 
                                    thickness = 22, 
                                    duration = -1.0, 
                                    is_persistent=True)
        
        
    def save_plot_lines(self, save_path):
        if len(self.plot_points) == 0:
            print('Empty plot points list. Empty File Saved.')
        else:
            with bz2.BZ2File(save_path, 'wb') as file: # compress file with bz2
                pickle.dump(self.plot_points, file) # Save it
            print('Plot points list saved successfully.')