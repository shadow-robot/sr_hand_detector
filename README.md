# sr_hand_detector

## Prerequisites

In order to use this package, install it from debian (recommended) or follow steps below:
1. Compile the package
2. Copy both executables of the sr_hand_detector package (found in `<your_workspace>/devel/lib/sr_hand_detector`) to `/usr/local/bin`.
3. Give one of the executables capability to access ethernet devices:
```sh
sudo setcap cap_net_raw+ep sr_hand_detector_node
```

## sr_hand_detector class

This class allows user to detect Shadow Hands without knowing the ethernet interface or the hand serial. Usage:

```sh
sr_hand_detector_node
```

Example output:
```sh
Detected hand on port: enx000ec653b31a
Hand's serial number: 634
```

Apart from the console output, all detected hand ethernet port names together with corresponding hand serial numbers will be set inside of the `/tmp/sr_hand_detector.yaml` file.

If there are no hands detected on any of the ports, a warning will be shown:
```sh
No hand detected on any of the ports!
```

## sr_hand_autodetect class

This class allows user to run Shadow Robot launch files without providing information like hand serial, ethercat port or hand side. Example usage:

```sh
sr_hand_autodetect roslaunch sr_robot_launch srhand.launch sim:=false
```

which is equivalent to:
```sh
roslaunch sr_robot_launch srhand.launch sim:=false eth_port:=<eth_port> hand_serial:=<hand_serial> side:=<hand_side> hand_type:=<hand_type> mapping_path:=<mapping_path>
```

If there are two hands connected and user wants to run only one of them using autodetection, there are `--right-only` (or `-r`) and `--left-only` (or `-l`) flags available, e.g.:

```sh
sr_hand_autodetect --right-only roslaunch sr_robot_launch srhand.launch sim:=false
```