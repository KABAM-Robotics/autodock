This package consolidates all configs related to the `kabam` project. The configs are just kept private here. The actual algorithm is open sourced here: https://github.com/osrf/autodock

### Run on Vacuum40

Run the `autodock_node` on the `Vacuum40` robot. 
Please make sure that the remote rosmaster time is in-synced.

```bash
export ROS_HOSTNAME=10.7.5.155 # your local ip
export ROS_MASTER_URI=http://10.7.5.88:11311
roslaunch autodock_kabam vacuum40.launch
```

**Faking ros clock with sim time (Caution)**

Here, the `autodockServer` node is running on a secondary machine, sharing the 
same roscore of the host. However here the host's time is not in sync with the 
secondary PC. (due to NTP server issue, and we cant ssh into the primary PC). 
Therefore to solve this headache, we will be publish the republish the primary pc 
clock to `clock`, this is handled by the `camera_info_filter.py` node. Then, 
enable `auto_docking_server` "fake clock" mode by adding a `--fake_clock` arg tag. 
Under the hood, this enables `use_sim_time` to only `auto_docking_server`.

---

### Run on Oscar

Add `gaussian 10.7.5.88` to `/etc/hosts`

```bash
export ROS_HOSTNAME=10.7.5.165 # your local ip
export ROS_MASTER_URI=http://10.7.5.88:11311
roslaunch autodock_kabam oscar.launch
```

> Hack: User might face ros-time off-synced issue, this can be avoided by enabling `use_fake_time`:

```bash
roslaunch autodock_kabam oscar.launch use_fake_clock:=1
```

Here, an external `obstacle_observer` node is used to pause when obstacle is detected. 
The robot's `/local_costmap` is being used here.

Similarly, a dock test script is used here as a smoke test. It will `cmd_vel` the robot randomly 
in front of the charger and dock it repeatedly.
```bash
# indicate the number of attempt with the -c arg
# indicate charging station srv name with -cs
rosrun autodock_examples dock_robot_test.py -c 10 -cs /xnergy_charger_rcu/trigger_charging
```

## Version History

### v5.2.0 [2025-05-22]
- Merged from Bitbucket repository into GitHub repository
- Added Static TF launch file for dock camera
- Added launch file for static TF with autodocking
- Build Docker image from `Dockerfile` inside autodock-kabam-artifacts for autodock and undock, `autodock_v5.2.0` image is built from this Dockerfile

### v5.1.0 [2024-09-20]
- Added the parameter `enable_stop_charge` to enable and disable the stop charging service before undocking, specifically for use with the Hzi Hertz charger

### v5.0.0 [2024-03-11]
- Added a generic `autodock_config.yaml` file instead of using the old oscar.yaml
- Added a new launch file `autodock.launch` to pass in the config file `autodock_config.yaml`
- Remapped `/cmd_vel` ROS topic name to `/cmd_mutex/dock`
- Removed the static tf publisher because it should be published by the camera not by the autodocking package
- Added a new Dockerfile which is the same as the existing Dockerfile_Oscar but with roslaunch command removed
