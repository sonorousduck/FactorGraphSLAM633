# Running the voxel map graphing and the scan matching

`colcon build --symlink-install` while in the ros_ws folder. Then run `source install/setup.bash`. Last, run `ros2 launch scan_matching octomap_mapping.launch.py`




# Running spatio_temporal_voxel_layer

## Building Spatio Temporal Voxel Layer

(I added all of this into the docker container now instead)

Run `sudo apt install ros-humble-spatio-temporal-voxel-layer`. It is way faster than compiling from the Github (that took me 35 min to build)

<!-- This takes forever, without adding the following export, it would spike my memory past taking 32GB of RAM and crash the build process.
You can try it without the flag and see if it works for you though! If it doesn't, add the tag. This is only necessary for the openvdb_vendor package. (This took 36min on my computer)

```
export MAKEFLAGS=-j1
colcon build --mixin release --symlink-install --parallel-workers 1 --packages-select openvdb_vendor
```

Afterwards, you will want to colcon build all the other packages. Since openvdb_vendor is already compiled, you will be good to just unset the MAKEFLAG and run the following

```
unset MAKEFLAGS
colcon build
``` -->


# Running rf2o_laser_odometry

I have already set the topic to listen to `/ouster/scan` and output on `/odom`. If you need that changed at all, the options are in the `rf2o_laser_odometry.launch.py` file.

```
Node(
    package='rf2o_laser_odometry',
    executable='rf2o_laser_odometry_node',
    name='rf2o_laser_odometry',
    output='screen',
    parameters=[{
        'laser_scan_topic' : '/ouster/scan',
        'odom_topic' : '/odom',
        'publish_tf' : True,
        'base_frame_id' : 'base_link',
        'odom_frame_id' : 'odom',
        'init_pose_from_topic' : '',
        'freq' : 20.0}],
),
```

To run the node, simply run `ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py`


# Running lidar_transformer

This node takes the lidar point clouds and transforms them to all be in the reference frame of the top lidar (frame id of `os_lidar_top`).

### All 5 LiDARs
It subscribes to `/ouster_back/points`, `/ouster_front/points`, `/ouster_left/points`, `/ouster_right/points`, and `/ouster_top/points`.

It performs the transformations and publishes to `/transformed_lidar/ouster_back/points`, `/transformed_lidar/ouster_front/points`, `/transformed_lidar/ouster_left/points`, `/transformed_lidar/ouster_right/points`, and
`/transformed_lidar/ouster_top/points` all under the fixed frame of `os_lidar_top`.

### Only 1 LiDAR

In the param file (found at `ros2_ws/src/lidar_transformer/config/params.yaml`), comment out all of the topics (or at least the ones you don't want to use) under `lidar_topics: ` except for `/ouster_top/points`. You shouldn't need to change anything else in the param file. 

### Build and run

Build in the `ros2_ws` directory and source the setup file.

```
colcon build --symlink-install && source install/setup.bash
```

> Note: There is a depreciation warning when building this package. It should work just fine if you ignore it.

Launch the node using the following:

```
ros2 launch lidar_transformer lidar_transformer.launch.py viz:=true # viz is optional 
```
> Note: You may need to run `xhost +local:docker` in your host terminal (on uvuntu 24 it works at least) to allow rviz to be launched from within the container.

Then play the bag file for all 5 LiDARs (Named *all_lidars_test1*. Downloadable at link in *ros2_ws/bags/link_to_bags.txt*)

```
ros2 bag play <path_to_bag>
```

# Running Image Publisher Node

Build same as all other ros2 packages and source the install/setup.bash.

Place any images you need in the `ros_ws/src/image_publisher/images` directory. 

Run the node using the following with the name of the image as the name in the `images` directory:
```
ros2 launch image_publisher image_publisher_launch.py image_name:=<Image_Name.png>
```