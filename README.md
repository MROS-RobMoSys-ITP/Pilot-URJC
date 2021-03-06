# Pilot-URJC
This repo contains the first pilot prepared by the URJC. It is composed of several elements that are explained below:

# Table of Contents
1. [What's the Pilot Behavior?](#pilot-behavior)
2. [The dialog system](#dialog-system)
3. [How are the system modes integrated in the pilot?](#metacontroller-pilot)
4. [Pilot-URJC and MROS ecosystem installation](#pilot-urjc-and-mros-ecosystem-installation)
5. [Launching MROS ecosystem](#launching-mros-ecosystem)
  - [In Real TIAGo robot](#real-tiago-robot)
  - [Simulated turtlebot3](#launching-in-simulated-turtlebot3)
6. [Launching Pilot-URJC](#launching-pilot-urjc)

### Pilot Behavior

This module contains the components that control the robot's mission during the pilot. We have used BICA components to generate the robot's behavior.

A [BICA](https://github.com/IntelligentRoboticsLabs/BICA/tree/ros2) component is a ROS2 lifecycle node that can activate another BICA component by merely declaring it as a dependency. Besides, each BICA component can use behavior trees to implement its behavior, being able to declare the dependencies on each behavior tree leaf. In this way, we can have hierarchical behavior trees.

In this pilot, the mission controller uses a behavior tree to sequence the phases of the test. In some stages, the robot is made to navigate, and in others, it starts a dialogue. As the dialogs are also composed of several stages, other BICA components have been used to implement it, activated from a leaf of the mission tree's behavior tree.

### Dialog System

The dialogue system can use both the robot's audio and tablet to communicate with a human. The selection of the communication mode is carried out using the hri_mode parameter. If the hri_mode parameter is "audio" the audio is used. If the hri_mode parameter is "tablet", the robot tablet is used.

The dialogue system uses a [distributed graph](https://github.com/IntelligentRoboticsLabs/BICA/tree/ros2/bica_graph) that is used in BICA for many tasks. When a "say: XXX" arc is added between the robot and the person, the robot says XXX. If the arc is "ask: YYY", a dialogue takes place until YYY finds out.

### Metacontroller-pilot

This module contains a ROS node that simulates the contingencies in the specifications of this pilot. Besides, it is responsible for asking the mode-manager to change the system mode. This node presents a simple menu in which a contingency can be activated, and when it ends. The module also contains the configuration of the pilot system modes.

The modes are:

1. Normal.
2. Degraded: An essential component of the robot has failed (the laser sensor), and it must use another component (RGBD camera) to replace it and continues with its tasks.
3. Performance: The robot increases its speed and does not execute recovery behaviors to reduce the task's execution time.
4. Energy saving: The robot is running out of battery. The robot must slow down the speed, and bt_modes_navigator informs the operator about this contingency.
5. Slow: The robot slow down the speed.

The modes change next parameters in the components (shown in the figure), with some effects:

* bt\_modes\_navigator [bt\_xml\_file]: bt_modes_navigator changes in runtime the behavior tree to adapt to the current situation. In mode obstructed, the behavior tree only contains an action leaf to contact the operator. In mode Low_Battery, the behavior tree also includes an action leaf to contact the operator. In mode Lost, the behavior Tree only contains activations of recovery behaviors.
* Controller [max\_vel]: Each mode defines an adequate speed.
* HRI Controller [hri_mode]: This parameter selects if the communication must be carried out using the tablet, or the audio.

![pilot_overview](resources/pilot-urjc.png)

## Pilot-URJC and MROS ecosystem installation

  We will use **vcs-tool** to get the dependencies and packages. We assume that you have a ros2 workspace ([ros2_ws]), if you don't have one, just create it with:

  ```console
    source /opt/ros/foxy/setup.bash
    mkdir -p [path-to-your-ros2-ws]/src
  ```

### Pilot-URJC and its dependencies

  Fetch, build and install navigation2 stack:

  ```console
    sudo apt install ros-foxy-slam-toolbox ros-foxy-gazebo-ros-pkgs

    cd [ros2_ws]/src
    git clone https://github.com/MROS-RobMoSys-ITP/Pilot-URJC.git
    vcs import < Pilot-URJC/dependencies.repos
    cd ..
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
    colcon build --symlink-install
    
    NOTE: if the compilation fails because of it could not find some packages run again 
      rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
      
    source [ros2_ws]/install/setup.bash
  ```

### Turtlebot3

  Turtlebot3 is one of tests platforms. Ignore this if you are not using it.
  Fetch, build and install turtlebot3 packages:

  ```console
    cd [ros2_ws]/src
    vcs import < Pilot-URJC/turtlebot3.repos
    cd ..
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
    colcon build --symlink-install
  ```  

## Launching MROS ecosystem

This pilot has been tested on different platforms. Above we show how to run the demo in each one.

### Real TIAGo robot.

  In the TIAGO_specifications.pdf of this repository you can find a full description of the TIAGo robot.

  #### Before start:
  1. Make sure that the navigation, localization and map-server are switched off in the robot before start the demo.
  2. The shell windows that will launch the ros1_bridge and the ros1 components needs a correct [network configuration](http://wiki.ros.org/ROS/NetworkSetup), setting the ROS_IP and ROS_MASTER_URI environment variables.
  3. We have used rmw_cyclonedds_cpp and rmw_fastdds_cpp RMW_IMPLEMENTATION for the tests.

  4. **![ros1_bridges](https://github.com/fmrico/ros1_bridge/tree/dedicated_bridges)**:
  To launch the demo in real TIAGo we have to use the some bridges, at this moment TIAGo drivers are not migrated to ROS2.
  ```console
    ros2 run ros1_bridge twist_2_to_1
    ros2 run ros1_bridge scan_1_to_2
    ros2 run ros1_bridge imu_1_to_2
    ros2 run ros1_bridge tf_static_1_to_2
    ros2 run ros1_bridge tf_1_to_2
  ```

1. **Navigation launcher**:
  This launcher includes rviz, nav2, amcl, map-server, **[system-modes](https://github.com/micro-ROS/system_modes)**, etc.
  The **system-modes mode-manager** takes the modes description from `params/pilot_modes.yaml`.
  ```console
    ros2 launch pilot_urjc_bringup nav2_tiago_launch.py
  ```

### Launching in simulated turtlebot3

1. **Launch turtlebot3 world in gazebo sim**

    ```console
      export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:[ros2_ws]/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models:[ros2_ws]/src/Pilot-URJC/pilot_urjc_bringup/worlds/models
      export TURTLEBOT3_MODEL=${TB3_MODEL}
      ros2 launch pilot_urjc_bringup tb3_sim_launch.py
    ```
  **After the last command, the Gazebo simulator is running in background. Don't worry if no window is opened.**
2. **TB3 navigation launcher**

    This launcher includes rviz, nav2, amcl, map-server, **[system-modes](https://github.com/micro-ROS/system_modes)**, etc.
    The **system_modes mode_manager** takes the modes description from `params/pilot_modes.yaml`.

    ```console
      export TURTLEBOT3_MODEL=${TB3_MODEL}
      ros2 launch pilot_urjc_bringup nav2_turtlebot3_launch.py
    ```
  **RVIz opens, and the navigation system is waiting for the activation of the laser_driver. This activation will be made automatically by the dummy_metacontroller in [Launching Pilot-URJC](#launching-pilot-urjc) step. It is not necessary to set an initial robot position with the 2D Pose Estimate tool. When the laser_driver is up, the pose will be set automatically.**

### Launching in simulated turtlebot2

To install ROS (Melodic version), all the full documentation and steps to do during this proccess are in the next page:

http://wiki.ros.org/melodic/Installation/Ubuntu

1. **Turtlebot ROS1 Gazebo simulator:**

  Launch the turtlebot2 simulator and its sensors. If you don't have a launcher to do this, you can find an example [here](https://github.com/IntelligentRoboticsLabs/gb_robots/tree/simulator)

  ```console
    ros2 launch gb_robots sim_house.launch
  ```

2. **![ros1_bridge](https://github.com/ros2/ros1_bridge)**:
  ```   
    ros2 launch nav2_bringup nav2_tb3_system_modes_sim_launch.py
  ```
3. **[nav2-TIAGo-support](https://github.com/IntelligentRoboticsLabs/nav2-TIAGo-support)**:
  The integration of the nav2 and turtlebot2 through the ros1_bridge needs one tools to fix some issues.
  ```
    rosrun tf_static_resender tf_static_resender
  ```

4. **Navigation launcher**:
  This launcher includes rviz, nav2, amcl, map-server, **[system-modes](https://github.com/micro-ROS/system_modes)**, etc.
  The **system-modes mode-manager** takes the modes description from params/pilot_modes.yaml.
  ```
    ros2 launch pilot_urjc_bringup nav2_turtlebot2_launch.py
  ```

### Real Kobuki robot.
  #### Before start:
  ```console
    sudo apt-get install ros-eloquent-eigen*
    cd [ros2_ws]/src
    git clone https://github.com/MROS-RobMoSys-ITP/Pilot-URJC.git
    vcs import src < Pilot-URJC/dependencies_kobuki.repos # if you did vcs import of dependencies.repos before, you have to comment Behavior Tree and navigation2 in the dependencies_kobuki.repos file before do this vcs import.
    cd ..
    colcon build --symlink-install
  ```

#### Setup:
- Add the map of the scenario in the Pilot-URJC/pilot_kobuki/map.
- Add a little modification in navigation2/nav2_bringup/bringup/launch/nav2_bringup_launch.py:
```console
    kobuki_dir = get_package_share_directory('pilot_kobuki')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(kobuki_dir, 'map', 'map_name.yaml'),
        description='Full path to map yaml file to load')
```
- Set base_frame_id value in navigation2/nav2_bringup/bringup/params/nav2_params.yaml:

```
  base_frame_id: "base_link"
```

#### Test the navigation in the real Kobuki robot:
```console
  source [ros2_ws]/install/setup.bash
  ros2 launch pilot_kobuki kobuki2.launch.py
```

## Launching Pilot-URJC

1. **A dummy metacontroller**

    With this tool, you can simulate different reconfiguration scenarios. **By now, the battery contingency is the only one supported. The laser failure is managed by the system_modes package**

    ```console
      ros2 run metacontroller_pilot metacontroller
    ```
    
**When the metacontroller is bringing up, the laser_driver goes to active mode, and everything starts. RVIz shows the local and global costmaps, the laser measures, and the transform tree.**

Finally, we launch the pilot.

2. **Pilot Behavior**

    ```console
      ros2 launch pilot_behavior pilot_urjc_launch.py
    ```
    **The robot patrols the scenario and its battery is draining.**
    
3. **Laser failure sim**
![rviz_cont_tool](resources/contingency_tool.png)
With this RVIz tool, you can simulate a laser failure and its consequences.
