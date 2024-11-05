# ZV2 Metadata
This directory contains scripts to build and run a docker image to extract images with ZV2 metadata from a ROS bag.

## Recording a bag.
With ardupilot_sitl and the Airsim ROS2 node running, record a bag with all topics (replace the path and bag name):
```
cd /path/to/Airsim/ros2/zv2_metadata
ros2 bag record -a -o bag_name
```

# Build the image.
```
./build.sh
```

## Extract images with metadata from a bag (replace the bag name):
```
./run.sh bag_name
```

