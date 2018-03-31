<<<<<<< HEAD
# ros-orb-slam2

ROS Publisher Node using ORB-SLAM2.
Currently, ORB_SLAM2 offers build file for ROS, but it doesn't broadcast any message, so other packages can not utilize it.

## Implementation

- [X] Mono Camera Slam
- [ ] Stereo Camera Slam
- [ ] RGB-D Camera Slam

## Installation

### Build ORB_SLAM2

See [ORB_SLAM2 Github Repository](https://github.com/raulmur/ORB_SLAM2).

```
$ git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
$ cd ORB_SLAM2
$ chmod +x build.sh
$ ./build.sh
```

After building ORB_SLAM2, set the environment variable ORBSLAM2_HOME.

```
$ export ORBSLAM2_HOME='{path/to/your/orbslam2/home}'
```

### Caktkin Build 

```
$ cd src
$ git clone https://github.com/ildoonet/ros-orb-slam2
$ cd ..
$ catkin_make
```

## Message

### Parameter

+ camera(string, default: /camera/image) : camera image raw topic name
+ worldframe(string, default: /world) : world frame name
+ frame(string, default: orbframe) : tf broadcast frame name
+ path_vocabulary(string) : vocabulary path(eg. $(env ORBSLAM2_HOME)/Vocabulary/ORBvoc.txt)
+ path_settings(string) : orb slam setting path(eg. $(find orb_slam2)/conf/fpvcam.yaml)

### Broadcast

Using [Transform Broadcaster](http://wiki.ros.org/tf), this always broadcasts frame when this receives image frame and processed it.

## Example

### Launch File Example

```
<node name="orb_slam2" pkg="orb_slam2" type="orb_slam2_mono" output="screen" required="true">
        <param name="camera" value="/videofile/image_raw" />
        <param name="path_vocabulary" value="$(env ORBSLAM2_HOME)/Vocabulary/ORBvoc.txt" />
        <param name="path_settings" value="$(arg orbslam_setting_path)" />
        <param name="use_viewer" type="bool" value="true" />
    </node>
```

### Mono USB Camera Example

See [/launch/camtest.launch](https://github.com/ildoonet/ros-orb-slam2/blob/master/launch/camtest.launch).
=======
# ros_orb_slam2
>>>>>>> a80b094897e9a51f31cc52596becda626c157148
