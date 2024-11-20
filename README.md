# Running the voxel map graphing and the scan matching

`colcon build --symlink-install` while in the ros_ws folder. Then run `source /install/setup.bash`. Last, run `ros2 launch scan_matching octomap_mapping.launch.py`




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