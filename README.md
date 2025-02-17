# lidar_mirror_fov_reshaper

Further informations about the main project (OpenSeeGround, 2025) in which those packages were developed and used can be found in the [OpenSeeGround](https://github.com/ioskn/OpenSeeGround) Project.

## Documentation

### Installation Prerequisites

```bash
apt install doxygen # Documentation Generator
pip install -U sphinx breathe # Documentation Frameworks
pip install sphinx_rtd_theme # Documentation Theme
```

### Build & View Documentation

```bash
cd <path/to/lidar_mirror_fov_reshaper>
cd docs/sphinx
make html # build documentation as html
```

```bash
<browser_of_choice> <path/to/lidar_mirror_fov_reshaper>/docs/sphinx/build/html/index.html
```

### View Raw Doxygen Documentation

```bash
<browser_of_choice> <path/to/lidar_mirror_fov_reshaper>/docs/doxygen/build/html/index.html
```

## lidar_mirror_fov_reshaper_transformation

Library package containting operations to transform lidar rays (in euclidean space)

## lidar_mirror_fov_reshaper_calibration

Functionality to calibrate the attached mirror poses. Only supports usage of exactly two mirrors in use. Supports lidar sensors capable of measuring intensity values and those that are not. Calibration is based on a independent two-step approach: 1. Calibrate the mirror positions, relative to the lidar sensor 2. Calibrate the mirror orientations, relative to the lidar sensor

## lidar_mirror_fov_reshaper_runtime

Contains the runtime functionality to reshape the lidar field of view (FOV) based on the calibrated mirror poses. Supports several types of FOVs to be published, details can be found in the documentation and the corresponding [configuration file](lidar_mirror_fov_reshaper_runtime/config/params.yaml).

## License

All packages are distributed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0), 2025 Andreas Loeffler
