# nautluis_sim
ROS Packages for Nautilus Simulation

![Simulator on the left window, UI and terminal on the right windows](https://github.com/uwrov/nautilus_sim/blob/main/desktop.png?raw=true)

# Usage
Make sure you have docker installed and have access to [nautilus_surface](https://github.com/uwrov/nautilus_surface).

## Container Usage
### Build Environment
```
docker-compose -f local-compose.yaml --profile sim up --build
```

Or, if you don't want to rebuild your container

```
docker-compose -f local-compose.yaml --profile sim up
```

### Running with roslaunch
Start up the containers with docker compose then run this command on the surface container. Open the UI on http://localhost:3000 to enable controller input (works best on chrome).
```
roslaunch nautilus_launch sim.launch
```

Then, if you want the visualization of the sim through gzweb (on http://localhost:8080), run the following in the sim container
```
rosrun nautilus_worlds launch_web.bash
```

### Running with Individual Commands

**0. Launch ROS on the surface container**
  ```
  roscore
  ```

  or, if you want the surface code

  ```
  roslaunch nautilus_launch system.launch
  ```

  The rest of the commands all take place in the sim container.

**1. Start gzserver (in it's own shell)**
  ```
  roscd nautilus_worlds/worlds && rosrun gazebo_ros gzserver underwater.world --verbose
  ```

**2. Start gzweb (in it's own shell)**
  ```
  cd /root/gzweb && npm start
  ```

**3. Compile your URDF**

  For Nautilus
  ```
  roscd nautilus_description/urdf && xacro nautilus.urdf.xacro > nautilus.urdf
  ```

  For ROV 22
  ```
  roscd nautilus_description/urdf && xacro rov22.urdf.xacro > rov22.urdf
  ```

**4. Spawn your URDF**

  For Nautilus
  ```
  roscd nautilus_description/urdf && rosrun gazebo_ros spawn_model -f nautilus.urdf -urdf -model nautilus -z 2
  ```

  For ROV 22
  ```
  roscd nautilus_description/urdf && rosrun gazebo_ros spawn_model -f rov22.urdf -urdf -model rov22 -z 2
  ```

  This spawns the post-processed xacro file in gazebo. Gazebo will implictly convert from `urdf` to `sdf` at this time, if this step does not work then run `check_urdf <(xacro nautilus.urdf.xacro)` and see if there are any errors.

**5. (Alternativately) compile and spawn at the same time**

For ROV 22
```
roscd nautilus_description/urdf && xacro rov22.urdf.xacro > rov22.urdf && rosrun gazebo_ros spawn_model -f rov22.urdf -urdf -model rov22 -z 2
```

## Overview
This is a highly portable simulation of UWROV's ROV. It's capable of being used as a stand-in for actual hardware, and can give a rough idea of how our ROV might interact with the real world.

### nautilus_description
This package contains the physical description of the ROV, some key files to pay attention to:
- `urdf/nautilus.urdf.xacro`
  - A macro file which contains the physical definition of the ROV. Changes to motor layout or physical measurements should happen here.
- `urdf/nautilus.gazebo`
  - Another macro file which contains the plugin and sensor data which interacts with the ROV.
- `meshes/nautilus.dae`
  - Model of the ROV, used for visuals

### nautilus_worlds
Contains the world information and any custom plugins we write:
- `worlds/underwater.world`
  - A sdf file containing the definition of the world the ROV inhabits. Change or copy this file if you need to add any props in.
- `src/`
  - Contains all the custom plugins which power the simulation. The thruster managing code is here.
