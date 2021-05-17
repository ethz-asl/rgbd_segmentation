# ROS package for RGB-D segmentation

This package implements geometric-semantic segmentation of input RGB-D sequences. Incoming RGB frames are processed with [Mask R-CNN](https://github.com/ethz-asl/mask_rcnn_ros) to detect and recognize object instances and predict for each a segmentation mask. Corresponding depth frames are processed with a [geometric segmentation approach](https://github.com/ethz-asl/cloud_segmentation) which clusters 3D points via a region-growing strategy. The two segmentation results are combined by matching the extracted clusters to the predicted segmentation masks via a 2D overlap measure.

The code has been developed as part of the [**TSDF++** framework](https://github.com/ethz-asl/tsdf-plusplus) for multiple dynamic object tracking and reconstruction. 

## Citing 

When using this code in your research, please cite the following publication:

Margarita Grinvald, Federico Tombari, Roland Siegwart, and Juan Nieto, **TSDF++: A Multi-Object Formulation for Dynamic Object Tracking and Reconstruction**, in _2021 IEEE International Conference on Robotics and Automation (ICRA)_, 2021. [[Video](https://youtu.be/dSJmoeVasI0)]

```bibtex
@article{grinvald2021tsdf,
  author={M. {Grinvald} and F. {Tombari} and R. {Siegwart} and J. {Nieto}},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  title={{TSDF++}: A Multi-Object Formulation for Dynamic Object Tracking and Reconstruction},
  year={2021},
}
```

## Installation

The installation has been tested on Ubuntu 16.04 and Ubutnu 20.04.

### Prerequisites
Install ROS following the instructions at the [ROS installation page](http://wiki.ros.org/ROS/Installation). The full install (`ros-kinetic-desktop-full`, `ros-melodic-desktop-full`) are recommended. 

Make sure to source your ROS _setup.bash_ script by following the instructions on the ROS installation page.

### Requirements
- C++14 for [PCL 1.10](https://github.com/PointCloudLibrary/pcl)

### Installation on Ubuntu
In your terminal, define the installed ROS version and name of the catkin workspace to use:
```bash
export ROS_VERSION=kinetic # (Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic)
export CATKIN_WS=~/catkin_ws
```

If you don't have a [catkin](http://wiki.ros.org/catkin) workspace yet, create a new one:
```bash
mkdir -p $CATKIN_WS/src && cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION --merge-devel 
catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release
wstool init src
```

Clone the `rgbd_segmentation` repository over HTTPS (no Github account required) and automatically fetch dependencies:
```bash
cd $CATKIN_WS/src
git clone https://github.com/ethz-asl/rgbd_segmentation.git
wstool merge -t . rgbd_segmentation/rgbd_segmentation_https.rosinstall
wstool update
```

Alternatively, clone over SSH (Github account required):
```bash
cd $CATKIN_WS/src
git clone git@github.com:ethz-asl/rgbd_segmentation.git
wstool merge -t . rgbd_segmentation/rgbd_segmentation_ssh.rosinstall
wstool update
```

Build and source the TSDF++ packages:
```bash
catkin build
source ../devel/setup.bash # (bash shell: ../devel/setup.bash,  zsh shell: ../devel/setup.zsh)
```

## License
The code is available under the [MIT license](https://github.com/ethz-asl/rgbd_segmentation/blob/master/LICENSE).
