.. _lidar_mirror_fov_reshaper_runtime:

lidar_mirror_fov_reshaper_runtime
=================================

Usage
-------------------------------

.. code-block:: bash

    ros2 launch lidar_mirror_fov_reshaper_runtime lidar_mirror_fov_reshaper_runtime.launch.py


Parameters
--------------------------------
.. list-table:: lidar_mirror_fov_reshaper_runtime Configuration-Parameters
  :header-rows: 1
  :widths: 20 50 30

  * - Parameter Name
    - Description
    - Default Value
  * - laser_scanner.laser_topic
    - Topic used to capture incoming raw lidar data.
    - "cloud"
  * - laser_scanner.laser_frame
    - Reference frame in which the lidar data is going to be published/transformed in.
    - "cloud"
  * - laser_scanner.angle_min
    - Minimum scan angle of the lidar data.
    - -2.35619 (rad, double)
  * - laser_scanner.angle_max
    - Maximum scan angle of the lidar data.
    - 2.35619 (rad, double)
  * - laser_scanner.use_pointcloud_input
    - Use point cloud input. If false, use laserscan msg as input. 
    - true
  * - visualization.pub_front
    - Publish the front, unmirrored view in a separate topic.
    - true
  * - visualization.pub_transformed_left_mirror
    - Publish the left mirror transformed FOV, in a separate topic.
    - true
  * - visualization.pub_transformed_right_mirror
    - Publish the right mirror transformed FOV, in a separate topic.
    - true
  * - visualization.pub_transformed_all_combined
    - Publish the combined view, of unmirrored and mirrored FOV, in a separate topic.
    - true
  * - visualization.pub_front_topic
    - Topic used for the front, unmirrored FOV topic.
    - "front"
  * - visualization.pub_left_mirror_topic
    - Topic for the left mirror, transformed FOV topic.
    - "left_mirror"
  * - visualization.pub_right_mirror_topic
    - Topic for the right mirror, transformed FOV topic.
    - "right_mirror"
  * - visualization.pub_all_combined_topic
    - Topic for the combined view, of unmirrored and mirrored FOV topic.
    - "all_combined"
  * - visualization.remove_unused_points
    - Remove points (set to NaN) that are being removed/unused inbetween the FOVs (e.g. points with invalid range data).
    - true
  * - front.start_angle
    - Start angle for the front, unmirrored FOV.
    - -45 (deg, int)
  * - front.end_angle
    - End angle for the front, unmirrored FOV.
    - 45 (deg, int)
  * - mirror_left.start_angle
    - Start angle for the left mirror FOV.
    - 90 (deg, int)
  * - mirror_left.end_angle
    - End angle for the left mirror FOV.
    - 135 (deg, int)
  * - mirror_right.normal_vec
    - Normal vector for the right mirror, representing its orientation.
    - [0.39866, -0.33, -0.335313]
  * - mirror_right.support_vec
    - Support vector for the right mirror, representing its position relative to the lidar sensor.
    - [-0.028539, 0.070636, 0.0]
  * - mirror_right.start_angle
    - Start angle for the right mirror FOV.
    - -135 (deg, int)
  * - mirror_right.end_angle
    - End angle for the right mirror FOV.
    - -90 (deg, int)
  * - mirror_left.normal_vec
    - Normal vector for the left mirror, representing its orientation.
    - [0.361, 0.29, -0.291]
  * - mirror_left.support_vec
    - Start angle for the left mirror, representing its position relative to the lidar sensor.
    - [-0.031, -0.069, 0.0]



.. doxygenfile:: lidar_mirror_fov_reshaper_runtime.hpp
    :project: lidar_mirror_fov_reshaper_runtime