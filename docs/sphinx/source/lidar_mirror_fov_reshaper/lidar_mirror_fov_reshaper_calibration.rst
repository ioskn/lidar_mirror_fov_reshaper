.. _lidar_mirror_fov_reshaper_calibration:

lidar_mirror_fov_reshaper_calibration
=====================================

Usage
-------------------------------

.. code-block:: bash

    ros2 launch lidar_mirror_fov_reshaper_calibration lidar_mirror_fov_reshaper_calibration.launch.py

Parameters
--------------------------------
.. list-table:: lidar_mirror_fov_reshaper_calibration Configuration-Parameters
  :header-rows: 1
  :widths: 20 50 30

  * - Parameter Name
    - Description
    - Default Value
  * - laser_scanner.topic
    - Topic used to capture incoming raw lidar data.
    - "scan"
  * - laser_scanner.laser_frame
    - Reference frame in which the lidar data is going to be published/transformed in.
    - "cloud"
  * - laser_scanner.angle_min
    - Minimum scan angle of the lidar data.
    - -135 (deg, int)
  * - laser_scanner.angle_max
    - Maximum scan angle of the lidar data.
    - 135 (deg, int)
  * - laser_scanner.use_pointcloud_input
    - Use point cloud input. If false, use laserscan msg as input. 
    - true
  * - visualization.pub_transformed_all_combined
    - Publish the combined view, of unmirrored and mirrored FOV, in a separate topic.
    - true
  * - visualization.pub_transformed_right_mirror
    - Publish the right mirror transformed FOV, in a separate topic.
    - true
  * - visualization.pub_transformed_left_mirror
    - Publish the left mirror transformed FOV, in a separate topic.
    - true
  * - visualization.pub_front
    - Publish the front, unmirrored view in a separate topic.
    - true
  * - visualization.normal_vectors
    - Publish the resulting normal vectors of the mirrors and the calibration plane.
    - true
  * - visualization.optimized_planes_all
    - Publish the optimized calibration plane and mirrors.
    - true
  * - visualization.optimized_planes_rm
    - Publish the calibrated right mirror plane.
    - true
  * - visualization.optimized_planes_lm
    - Publish the calibrated left mirror plane.
    - true
  * - visualization.optimized_plane_opt_plane
    - Publish the optimized calibration plane.
    - true
  * - visualization.optimization_plane_box
    - Publish a box around the calibration plane in order to visualization sensor noise.
    - true
  * - optimization.filter_method
    - Filter method used to identify/find the target ref. point on the calibration plane. 0 = intensity filter, 1 = distance filter
    - 0
  * - optimization.filter_dist_threshold
    - The distance threshold used if filter_method = 1
    - 0.1
  * - optimization.interpolation_window
    - # Interpolate intensity threshold(-area) over this window size +-interpolation_window. Thus if 0, only interpolate highest intensity value idx. If -1 no interpolation is done
    - 0
  * - optimization.averaging_n_clouds
    - Number of clouds to be averaged for the optimization. If -1, all clouds are used, no averaging
    - -1
  * - optimization.intensity_threshold_percentage
    - percentage of the maximum intensity value to be used as threshold for the intensity filter
    - 0.5
  * - optimization.buffer_size
    - Number of scans used as an input for the calibration
    - 3500
  * - optimization.epsabs
    - The absolute error used as a stopping criterion in the optimization method.
    - 1e-4
  * - optimization.stepsize
    - The step size used in the optimization method.
    - 1e-3
  * - optimization.iter_max
    - The maximum number of iterations used as a stopping criterion in the optimization method.
    - 1000
  * - optimization.adaptive_stepsize
    - If true, the step size will be adaptive, and decrease over time.
    - false
  * - optimization.opt_mirror_orientation
    - If true, the mirror orientation will be optimized/calibrated.
    - true
  * - optimization.opt_mirror_support_vec
    - If true, the mirror support vector will be optimized/calibrated.
    - false
  * - optimization.write_optimized_params
    - If true, the optimized/calibrated parameters will be written to a file.
    - false
  * - optimization.write_optimization_history
    - If true, the optimization/calibration history will be written to a file.
    - false
  * - optimization.optimization_history_file
    - The file name of the optimization/calibration history. Assumed to be a .csv file.
    - "lmfrc_history.csv"
  * - optimization.optimized_params_meta_file
    - The file name of the optimization/calibration metadata. Assumed to be a .csv file.
    - "lmfrc_metadata.csv"
  * - optimization.optimized_params_file
    - The file name of the optimized/calibrated parameters. Assumed to be a .csv file.
    - "lmfrc_results.csv"
  * - optimization.optimized_params_out_dir
    - The directory in which the files related to the optimization/calibration will be stored. Assumes the write of either the optimized parameters or the optimization history.
    - "/my_out_dir/"
  * - optimization.verbose 
    - Level of verbosity. 0 = no output, 1 = minimal output, 2 = detailed output
    - 0
  * - optimization.evaluation_no_batches
    - Number of batches to be used for additional evaluation of the optimization/calibration results. 1 = No batch based evaluation.
    - 1
  * - optimization.optimization_mirror_orientation.opt_all
    - Shortcut to opt. all parameters during the mirror orientation optimization/calibration.
    - false
  * - optimization.optimization_mirror_orientation.opt_mirror_svs
    - Shortcut to opt. all mirror support vector parameters during the mirror orientation optimization/calibration.
    - true
  * - optimization.optimization_mirror_orientation.opt_mirror_nvs
    - Shortcut to opt. all mirror normal vector parameters during the mirror orientation optimization/calibration.
    - true
  * - optimization.optimization_mirror_orientation.opt_plane_sv
    - Shortcut to opt. all plane support vector parameters during the mirror orientation optimization/calibration.
    - true
  * - optimization.optimization_mirror_orientation.opt_osg_settings
    - Shortcut to opt. parameters according to the original openseeground settings during the mirror orientation optimization/calibration.
    - true
  * - optimization.optimization_mirror_orientation.right_mirror.support_vec.x
    - Modify the right mirrors support vector x component during the optimization/calibration of its orientation.
    - false
  * - optimization.optimization_mirror_orientation.right_mirror.support_vec.y
    - Modify the right mirrors support vector y component during the optimization/calibration of its orientation.
    - false
  * - optimization.optimization_mirror_orientation.right_mirror.support_vec.z
    - Modify the right mirrors support vector z component during the optimization/calibration of its orientation.
    - false
  * - optimization.optimization_mirror_orientation.right_mirror.normal_vec.x
    - Modify the right mirrors normal vector x component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.right_mirror.normal_vec.y
    - Modify the right mirrors normal vector y component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.right_mirror.normal_vec.z
    - Modify the right mirrors normal vector z component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.left_mirror.support_vec.x
    - Modify the left mirrors support vector x component during the optimization/calibration of its orientation.
    - false
  * - optimization.optimization_mirror_orientation.left_mirror.support_vec.y
    - Modify the left mirrors support vector y component during the optimization/calibration of its orientation.
    - false
  * - optimization.optimization_mirror_orientation.left_mirror.support_vec.z
    - Modify the left mirrors support vector z component during the optimization/calibration of its orientation.
    - false
  * - optimization.optimization_mirror_orientation.left_mirror.normal_vec.x
    - Modify the left mirrors normal vector x component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.left_mirror.normal_vec.y
    - Modify the left mirrors normal vector y component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.left_mirror.normal_vec.z
    - Modify the left mirrors normal vector z component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.calibration_plane.support_vec.x
    - Modify the calib.plane support vector x component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.calibration_plane.support_vec.y
    - Modify the calib.plane support vector y component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.calibration_plane.support_vec.z
    - Modify the calib.plane support vector z component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.calibration_plane.normal_vec.x
    - Modify the calib.plane normal vector x component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.calibration_plane.normal_vec.y
    - Modify the calib.plane normal vector y component during the optimization/calibration of its orientation.
    - true
  * - optimization.optimization_mirror_orientation.calibration_plane.normal_vec.z
    - Modify the calib.plane normal vector z component during the optimization/calibration of its orientation.
    - true
  * - calibration_plane.support_vec
    - Initial support vector of the calibration plane.
    - [0.15, 0.0, -0.35]
  * - calibration_plane.helper_p1 
    - Helper vector used to calculate the calibration planes orientation.
    - [0.15, 0.35, 0.0]
  * - calibration_plane.helper_p2
    - Helper vector used to calculate the calibration planes orientation.
    - [0.15, -0.2, 0.0]
  * - calibration_plane.helper_p3
    - Helper vector used to calculate the calibration planes orientation.
    - [0.15, 0.0, 0.1]
  * - mirror_front.start_angle
    - The start angle of the mirror front.
    - -45
  * - mirror_front.end_angle
    - The end angle of the mirror front.
    - 45
  * - mirror_left.mirror_safety_bufferzone_size
    - The size of the mirror safety buffer zone.
    - 1
  * - mirror_left.auto_define_start_angle
    - If true, the start angle will be automatically defined.
    - false
  * - mirror_left.auto_define_end_angle
    - If true, the end angle will be automatically defined.
    - false
  * - mirror_left.auto_define_angle_mode
    - 0 = via slope of distance values; 1 = avg_distance threshold; ONLY applied if auto_define_{start||end}_angle == true
    - 0
  * - mirror_left.start_angle
    - If auto_define_{start||end}_angle == true, this value is being used as an initial guess
    - -45
  * - mirror_left.end_angle
    - If auto_define_{start||end}_angle == true, this value is being used as an initial guess
    - 45
  * - mirror_left.helper_p1
    - The first helper point of the left mirror.
    - [-0.018, 0.057, -0.01846]
  * - mirror_left.helper_p2 
    - The second helper point of the left mirror.
    - [-0.042, 0.045, 0.01846]
  * - mirror_left.helper_p3
    - The third helper point of the left mirror.
    - [-0.015, 0.057, 0.01475]
  * - mirror_left.support_vec
    - The support vector of the left mirror.
    - [0.0, 0.094, 0.0]
  * - mirror_left.normal_vec
    - The normal vector of the left mirror.
    - [0.45, -0.33, -0.403583]
  * - mirror_right.mirror_safety_bufferzone_size
    - The size of the mirror safety buffer zone.
    - 1
  * - mirror_right.auto_define_start_angle
    - If true, the start angle will be automatically defined.
    - true
  * - mirror_right.auto_define_end_angle
    - If true, the end angle will be automatically defined.
    - true
  * - mirror_right.auto_define_angle_mode
    - 0 = via slope of distance values; 1 = avg_distance threshold; ONLY applied if auto_define_{start||end}_angle == true
    - 0
  * - mirror_right.start_angle
    - If auto_define_{start||end}_angle == true, this value is being used as an initial guess
    - -135
  * - mirror_right.end_angle
    - If auto_define_{start||end}_angle == true, this value is being used as an initial guess
    - -90
  * - mirror_right.helper_p1
    - The first helper point of the right mirror.
    - [-0.018, -0.057, -0.01846]
  * - mirror_right.helper_p2
    - The second helper point of the right mirror.
    - [-0.042, -0.045, 0.01846]
  * - mirror_right.helper_p3
    - The third helper point of the right mirror.
    - [-0.015, -0.057, 0.01475]
  * - mirror_right.support_vec
    - The support vector of the right mirror.
    - [0.0, -0.095, 0.0]
  * - mirror_right.normal_vec
    - The normal vector of the right mirror, representing its orientation. If a dedicated normal vector is not provided (all entries are 0), the normal vector will be calculated based on the helper points/vectors.
    - [0.4, 0.29, -0.33]


.. doxygenfile:: lidar_mirror_fov_reshaper_calibration.hpp
    :project: lidar_mirror_fov_reshaper_calibration