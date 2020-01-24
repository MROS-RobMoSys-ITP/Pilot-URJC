# Pilot-URJC

This repo contains the first pilot prepared by the URJC. It is composed of several elements that are explained below:

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
2. Low_Battery: The robot is running out of battery. The robot must slow down the speed, and bt_modes_navigator informs the operator about this contingency.
3. Obstructed: The robot finds an obstacle, and it can't find an alternative path. The robot informs the operator about this contingency.
4. Lost: The localization information has degraded. The robot runs recovery behaviors.
5. Network_Down: Network is down, so the dialog must be carried out using the tablet instead of DialogFlow, which requires a network connection.

The modes change next parameters in the components (shown in the figure), with some effects:

* bt\_modes\_navigator [bt\_xml\_file]: bt_modes_navigator changes in runtime the behavior tree to adapt to the current situation. In mode obstructed, the behavior tree only contains an action leaf to contact the operator. In mode Low_Battery, the behavior tree also includes an action leaf to contact the operator. In mode Lost, the behavior Tree only contains activations of recovery behaviors.
* Controller [max\_vel]: Each mode defines an adequate speed.
* HRI Controller [hri_mode]: This parameter selects if the communication must be carried out using the tablet, or the audio.


![pilot_overview](resources/pilot-urjc.png)