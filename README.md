# Description
This repository contains a python implementation of the Bug2 algorithm for path-planning and obstacle-avoidance of a multirotor in the AirSim simulator.


## Introduction to Bug Algorithms
Bug algorithms belong to a category of path-planning methods that take inspiration from maze-solving algorithms. They are specifically designed to navigate in environments with no knowledge about obstacles and rely solely on the relative positioning of their target. These algorithms move towards their destination by following the boundaries of obstacles they encounter.
There are several variations of the Bug algorithms that currently exist [[1]](#1):

1. **Com**: *Common Sense* algorithm, abbreviated as *Com*, initiates by tracing the shape of the object, but it quickly deviates from its path when it has the opportunity to head straight towards the goal. In general, this algorithm doesn't ensure that the robot will reach its destination successfully.

2. **Bug 1**: When confronted with an obstacle, Bug 1 starts by exploring the obstacle's entire border while simultaneously figuring out the nearest position to the target. After reaching this initial point of contact, Bug 1 moves towards the closest position to the target and then deviates from the obstacle at that specific point. Bug 1 excels in environments where Com's performance falls short.

3. **Bug 2**: Bug 1 tends to generate longer paths than necessary because it needs to map the entire border of the obstacle. However, Bug 2 introduces a new concept called M-line, which is an imaginary line drawn between the starting and target positions. Bug 2 follows along the obstacle's border until it reaches the same M-line on the opposite side. If the point on this M-line is closer to the target than where it currently hit the obstacle, Bug 2 will then navigate away from it.

<div style="text-align: center;">
  <img src="./Images/Bugs Example 1.PNG" alt="Bugs_1" width="800" height="276">
  <p>The behavior of simple Bug Algorithms.</p>
</div>

<div style="text-align: center;">
  <img src="./Images/Bugs Example 2.PNG" alt="Bugs_1" width="800" height="291">
  <p>Generated paths by Bug Algorithms.</p>
</div>


## Requirements
The code is written in Python 3.8 and has been tested on Windows 10 without any issues. It utilizes the following libraries:
<pre>
airsim==1.8.1
numpy==1.22.0
</pre>

Additionally, `Unreal Engine 4.27` is required to simulate the environment.

## Usage
1. Clone the repository.
2. Navigate to the `./AirSim_Settings` directory, then copy the `settings.json` file and paste it into the `./Users/YourUserName/Documents/AirSim` directory.
3. Download the environment precompiled binaries from [here](https://drive.google.com/file/d/1TtDHg56eYTOMozV_2vU-JxoXZXwSdzbT/view?usp=sharing).
4. Unzip the downloaded file, then go to the `./Environment Bug2/WindowsNoEditor` directory and launch `Environment_1.exe`.
5. Navigate to `./Bug2` directory and run `AutoPilot.py` and enjoy the simulation.

Please note that each run may **not** always successfully complete due to potential noise in the quadcopter's movement and sensors. If a run fails, consider re-running the code.

#### Multirotor Trajectory Visualization
In the `AutoPilot.py` script, the default setting for the `save_plot_list` variable is `True`. After a successful run, a compressed pickle file named `path_plot_points.pbz2` is generated in the `./Bug2/Saved` directory. You can visualize the trajectory points in two ways:
1. Set the `plot_path_planning` variable from `False` to `True` in the script. This will continuously plot the trajectory points in real-time as the multirotor moves towards its target destination.
2. Go to the `./AirSim_Settings` directory. Copy the `settings_computervision.json` file and paste it into the `./Users/YourUserName/Documents/AirSim` directory. Rename the file from `settings_computervision.json` to `settings.json`. Finally, run `./Bug2/PathPlot.py` to plot the saved trajectory points separately. You can control the camera using the keyboard's A, W, S, D, and arrow keys.


## Showcase
You can view examples of movement through the following GIFs, providing a visual showcase of navigation.
<div style="display: flex;">
  <img src="./Images/GIF 1.gif" width="31%" />
  <img src="./Images/GIF 2.gif" width="31%" />
  <img src="./Images/GIF 3.gif" width="31%" />
</div>


#### Video Demonstration
For a more in-depth demonstration, please visit [this link](https://youtu.be/486aIvB4I_c) to observe the multirotor navigation in action within the AirSim simulator.



## Acknowledgement
This repository is inspired by the work of *Mohammad Hashemi*, a fellow alumnus who has implemented the Bug algorithms on a 3-wheel omnidirectional robot in the [Webots](https://cyberbotics.com/) simulator. You can find his work [here](https://github.com/mohammadhashemii/Bug-Algorithms-Simulation).


## References

- <a id="1">[1]</a> [K. N. McGuire, G. C. de Croon, and K. Tuyls, “A comparative study of bug algorithms for robot navigation,” Robotics and Autonomous Systems, vol. 121, p. 103261, 2019.](https://doi.org/10.1016/j.robot.2019.103261)

- The environment utilized in this project is the *Modular Building Set*, which can be obtained from this [link](https://www.unrealengine.com/marketplace/en-US/product/modular-building-set/). The environment also incorporates packages such as the *Vehicle Variety Pack* available [here](https://www.unrealengine.com/marketplace/en-US/product/bbcb90a03f844edbb20c8b89ee16ea32) and the *Modular Neighborhood Pack* accessible [here](https://www.unrealengine.com/marketplace/en-US/product/modular-neighborhood-pack).

