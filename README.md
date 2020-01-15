# ouijabot

The ouijabot is an omnidirectional ground robot platform designed for general experiments in the plane, including 2D SLAM, collaborative manipulation, and pursuit-evasion. The platform's holonomic dynamics allow it to be well-modeled with a single-integrator model, which make it ideal for a wide variety of planar control and navigation tasks.

The hardware consists of 4 DC motors, which drive omnidirectional wheels, and an embedded Linux computer (Raspberry Pi). This project aims to completely open source all software and hardware. All communication is performed using ROS.

## Supported functions
Below are the features or capabilities of the ouijabot which
are currently supported.

---

### Onboard functions:
#### Velocity control
* Open-loop controller which seeks to track a desired robot velocity
  (combined linear and angular velocity).

#### Force control
* Closed-loop controller, which uses onboard current sensors to
  generate desired wrench (force and torque).

#### Sensors
* Measures current supplied to each motor by the motor drivers.
* Measures current linear acceleration and angular velocity via onboard IMU.

---
### Offboard functions:
#### Velocity teleoperation:
* ROS node which generates and publishes body-frame velocity
  commands via joystick input.

#### Force teleoperation:
* ROS node which generates and publishes body-frame force and torque commands
  via joystick input.


## Basic usage
Below we provide general instructions for how to utilize the supported features detailed above. Please see the code for additional details. All nodes typically run in a namespace set by the user's `$HOSTNAME` variable.

---

#### Velocity control
* **Onboard:** run `roslaunch ouijabot velocity_control.launch` to start the velocity control node (`velocity_control.py`). The node will subscribe to the topic `cmd_vel`, which uses messages of type `geometry_msgs/Twist`.
  * **Arguments:**
    * `cmdMode`: flag for desired throttle response of ouijabot. If set to `"equal"`, command axes are decoupled; throttle commands in one axis always correspond to the same speed, regardless of other axis commands. If set to `"max"`, then any command with l1-norm >= 1 is treated as "full speed"; at least one motor will run at max RPM. Allows for faster speeds overall of the ouijabot, but with less physical intuition behind commands.


* **Offboard:** to teleoperate ouijabot, run `roslaunch ouijabot_telop vel_telop.launch` to launch the velocity command node. The node will read joystick inputs and use them to generate velocity commands, which it publishes to `cmd_vel`.
  * **Arguments:**
    * `id`: index of ouijabot to be commanded. The node will remap the generic `cmd_vel` topic to `/ouijabot$(arg id)/cmd_vel` by default. Defaults to `1`.
    * `velMax_l`: scaling parameter for linear velocity commands. Should range between `0` and `1`, with `1` corresponding to max throttle available in hardware. Defaults to `1.0`.
    * `velMax_a`: scaling parameter for angular velocity commands. Same behavior as linear velocity scaling; defaults to `1.0`.
---
#### Force control
* **Onboard:** run `roslaunch ouijabot force_control.launch` to launch the force control node (`force_control.py`). The node will subscribe to the topic `cmd_wrench` which uses messages of type `geometry_msgs/Twist`. It uses onboard current sensors to run a closed-loop PID controller to generate the desired wrench.

    __*Note:*__ *`params.launch` must be run before `force_control.launch`; the PID parameters are set as ROS parameters in this file. This is typically done once, offboard, as part of robot setup. These parameters may easily be tuned by setting the ROS parameters manually.*

* **Offboard:** run `roslaunch ouijabot params.launch`, once, to set force controller parameters. Then run `roslaunch ouijabot_telop force_telop.launch` to teleoperate the ouijabot in force control mode. The node will read joystick inputs and use them to generate wrench commands, which it publishes to `cmd_wrench`.
  * **Arguments:**
    * `id`: index of ouijabot to be commanded. The node will remap the generic `cmd_vel` topic to `/ouijabot$(arg id)/cmd_vel` by default. Defaults to `1`.
    * `force_scale`: scaling parameter for max force command, in Newtons. Defaults to `100.0`.
    * `torque_scale`: scaling parameter for max torque command, in Newton-meters. Defaults to `5.0`.

---

#### Sensor readings
* **Current sensors:** run `roslaunch ouijabot current_read.launch` to run the current sensor node `current_read.py`. The node reads the current sensors on the motor controllers and publishes the values to the topic `current` as a vector of floats.

    __*Note:*__ *Do not run `current_read.py` at the same time as `force_control.py`; the force control node reads/publishes the current sensors as part of its control loop and the conflicting reads could cause timing errors.

* **IMU:** run `roslaunch ouijabot imu_read.launch` to run the IMU node `imu_read.py`. The node will read measurements from the IMU over I2C and publish them to the topic `imu` as messages of type `sensor_msgs/Imu`.
