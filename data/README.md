# Dataset installation

This project uses sequence `urban35` from the Complex Urban Dataset:

https://sites.google.com/view/complex-urban-dataset

Dataset files are not committed to Git and must not be included in course
submission archives. Place the extracted sequence at:

```text
data/complex_urban/urban35/
├── calibration/
├── image/
│   ├── stereo_left/
│   └── stereo_right/
├── sensor_data/
│   ├── encoder.csv
│   ├── xsens_imu.csv
│   ├── fog.csv
│   ├── gps.csv
│   ├── vrs_gps.csv
│   ├── VLP_left/
│   ├── VLP_right/
│   ├── SICK_back/
│   └── SICK_middle/
├── global_pose.csv
└── sick_pointcloud.las
```

Known sequence details:

- sensor streams have different rates and must be matched by timestamp;
- the first right stereo frame corresponding to `1544686261456267672` is absent;
- the first expected right VLP frame corresponding to `1544686261584782000` is absent;
- VRS-GPS epochs without a valid RTK fix are excluded from primary metrics.
