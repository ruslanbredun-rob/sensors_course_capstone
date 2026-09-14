# GPS-denied localization on Complex Urban Dataset

Course capstone project for evaluating 2D vehicle localization without GPS. The
baseline fuses wheel encoders and Xsens IMU in an EKF. Two extended configurations
add stereo visual odometry and LiDAR odometry.

## Planned experiments

| Configuration | Measurements used by the estimator |
|---|---|
| Base | Wheel encoders + Xsens IMU |
| Visual | Wheel encoders + Xsens IMU + stereo visual odometry |
| Full | Wheel encoders + Xsens IMU + stereo visual odometry + LiDAR odometry |

VRS-GPS is reserved as the independent absolute position reference and is not
used by the GPS-denied estimator. Position metrics are computed only at epochs
with a valid RTK fix. FOG and `global_pose.csv` may be used for secondary analysis;
`global_pose.csv` is not treated as independent ground truth.

## Repository layout

```text
config/                    experiment and dataset configuration
data/                      local dataset and download instructions
docs/design_document.md    homework 17 working document
docs/course_requirements/  local copies of homework 17-20 instructions
src/                       localization and evaluation modules
results/                   generated metrics and plots
slides/                    defense presentation
```

The implementation and reproducible run command will be added during homework 18.
See `data/README.md` for the local dataset layout.
