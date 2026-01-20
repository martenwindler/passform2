# PassForM ROS

This repository stores all ROS2-related code for PasSForM.
The initial PassForM project ended in 2023 creating the release version [v1.0](https://gitlab.ips.biba.uni-bremen.de/passform/passform_ros/-/tags/v1.0) (main branch) and [v1.0-beta](https://gitlab.ips.biba.uni-bremen.de/passform/passform_ros/-/tags/v1.0-beta) (develop branch).

## Getting started

Dedicated bringups are located in the [BIBA_modules](https://gitlab.ips.biba.uni-bremen.de/passform/modules_BIBA) repo.

Some examples are listed below.
Use `use_rviz:=False` to prevent the RVIZ GUI from showing up.
This saves a lot of ressources.

### Inventory example

Two material modules with each two compartments.

    ros2 launch passform_bringup mat.launch.py use_rviz:=False

Add items to the storage with

```
ros2 action send_goal /module_efc21a5b_471d_4f8e_975c_5967ef78255c/action/store passform_msgs/action/Passform '{task: {required_parameter: [{name: item_id, value: {type: 4, string_value: uuid}}, {name: quantity, value: {type: 2, integer_value: 12}}, {name: location_id, value: {type: 4, string_value: uuid}}], request_id: 12}}'
```

Use items with

```
ros2 action send_goal /module_efc21a5b_471d_4f8e_975c_5967ef78255c/action/provide passform_msgs/action/Passform '{task: {required_parameter: [{name: item_id, value: {type: 4, string_value: uuid}}, {name: quantity, value: {type: 2, integer_value: 12}}, {name: location_id, value: {type: 4, string_value: uuid}}], request_id: 12}}' 
```

### Operator

Operator with a websocket connection to show instructions on external GUIs.

    ros2 launch passform_bringup operator.launch.py use_rviz:=False

A simple skill can be sent with

```
ros2 action send_goal /perform_action passform_msgs/action/Passform '{task: {type: {skill_type: 1}, optional_parameter: [{name: uuid, value: {type: 4, string_value: 'test'}}]}}'
```

### Behavior

The operator is a good example since they can perform many actions.
Launch the operator setup as written above.

    ros2 launch passform_bringup operator.launch.py use_rviz:=False

Open a second terminal and run the behavior example

    ros2 launch passform_behavior process.launch.py

The process can be observed from Groot, connected in monitoring mode.

![Groot Example](docs/images/groot_example.png)

### Full example

    ros2 launch passform_bringup example.launch.py

A somehow complicated demo can be started with (each in its own terminal)

    ros2 launch base_bringup bringup.launch.py

    ros2 launch robot_module_bringup param.launch.py config_file:=params1.yaml

    ros2 launch material_module_bringup param.launch.py config_file:=params2.yaml

    ros2 launch material_module_bringup param.launch.py config_file:=params3.yaml

    ros2 launch robot_module_bringup param.launch.py config_file:=params4.yaml
    
    ros2 launch conveyor_module_bringup param.launch.py config_file:=params5.yaml

    ros2 launch conveyor_module_bringup param.launch.py config_file:=params6.yaml

    ros2 launch conveyor_module_bringup param.launch.py config_file:=params7.yaml

    ros2 launch conveyor_module_bringup param.launch.py config_file:=params8.yaml

Show the demo with

    ros2 launch passform_viz viz.launch.py

## Setup

### BaSyx

BaSyx runs in provided docker containers.
Register and start the containers as specified on [their website](https://wiki.eclipse.org/BaSyx_/_Documentation_/_Components).

Direct link to the used components:
- [Registry](https://wiki.eclipse.org/BaSyx_/_Documentation_/_Components_/_Registry)
- [Server](https://wiki.eclipse.org/BaSyx_/_Documentation_/_Components_/_AAS_Server)

The default CORS settings prevent access from other machines.
You can either use local config files or change the files inside the docker containers.

Start the container and get the container ID by

    docker ps

Copy the ```context.properties``` to your local machine with

    docker cp <CONTAINER_ID>:/usr/share/config/context.properties ~/

Modify the last line to allow access as required and copy it back to the container with

    docker cp ~/context.properties <CONTAINER_ID>:/usr/share/config/