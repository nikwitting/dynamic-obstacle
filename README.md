## HOW TO RUN
1. dts devel build -f --arch arm32v7 -H daringduck.local in the dynamic-obstacle folder

2. In 4 Terminals run the following:

docker -H daringduck.local run -it --privileged --rm -v /data:/data --net=host duckietown/dt-duckiebot-interface:daffy

docker -H daringduck.local run --privileged -it --rm --net host duckietown/dt-car-interface:daffy

dts duckiebot keyboard_control daringduck --base_image duckietown/dt-core:daffy-amd64

docker -H daringduck.local run -it --rm --net=host -v /data:/data -e OFFSET=0 --privileged duckietown/dynamic-obstacle:lanefollowing-arm32v7

(OFFSET 0: no offset, 1: left lane, 2, middle of the road, default=0)
3. press 'a' in the keyboard terminal to start lane following and 's' to stop.


## Implement perception part in package vehicle_detection/src/vehicle_detection_node.py, also look at vehicle_filter_node.py

## Implement control part in package vehicle_detection/src/vehicle_avoidance_control_node.py




#Template: template-ros

This template provides a boilerplate repository
for developing ROS-based software in Duckietown.

**NOTE:** If you want to develop software that does not use
ROS, check out [this template](https://github.com/duckietown/template-basic).


## How to use it

### 1. Fork this repository

Use the fork button in the top-right corner of the github page to fork this template repository.


### 2. Create a new repository

Create a new repository on github.com while
specifying the newly forked template repository as
a template for your new repository.


### 3. Define dependencies

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py.txt` (apt packages and pip packages respectively).


### 4. Place your code

Place your ROS packages in the directory `/packages` of
your new repository.

**NOTE:** Do not use absolute paths in your code,
the code you place under `/packages` will be copied to
a different location later.


### 5. Setup the launchfile

Change the file `launch.sh` in your repository to
launch your code.
