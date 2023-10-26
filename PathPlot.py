import bz2
import pickle
import airsim


# Connect to the quadcopter
client = airsim.MultirotorClient()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync(timeout_sec=0.2).join()



# Plot the path
load_path = "./Saved/ModularBuildingSet/path_plot_points.pbz2"
with bz2.BZ2File(load_path, 'rb') as file: # Load the data using pickle
    data = pickle.load(file)
client.simPlotLineList(points = data, 
                       color_rgba = [1.0, 0.0, 0.0, 1.0], 
                       thickness = 22, 
                       duration = -1.0, 
                       is_persistent=True)