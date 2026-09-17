# GPS-denied vehicle localization

Planar vehicle localization on the Complex Urban Dataset, sequence `urban35`.
The current `base` mode fuses wheel encoder speed with Xsens IMU acceleration
and yaw rate in an EKF. VRS-GPS is reserved for evaluation and is not an
estimator input.

## Dataset

Download `urban35` from the
[Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset)
and extract it to `data/complex_urban/urban35/`. The extracted sequence is
about 5.5 GB and is excluded from the repository. The required files for
`base` are:

```text
data/complex_urban/urban35/
├── calibration/
│   ├── EncoderParameter.txt
│   ├── Vehicle2IMU.txt
│   └── Vehicle2VRS.txt        # only for --validate
└── sensor_data/
    ├── encoder.csv
    ├── xsens_imu.csv
    └── vrs_gps.csv            # only for --validate
```

See [data/README.md](data/README.md) for the remaining dataset structure.

## Run

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py --validate
```

Run these commands from the project root. Use `--dataset PATH` and
`--output PATH` to override the default paths in
`config/default.json`. Use `--max-events 1000` for a short run. The default
mode is `base`; omit `--validate` to run only the estimator. Other mode names
are reserved but are not implemented.

`results/estimated_state.csv` contains one state after each IMU prediction or
wheel update. Columns include the nanosecond timestamp, event source, position
in metres, yaw in radians, speed in m/s, wheel NIS and update acceptance.
`results/run_summary.txt` contains event counts and the final local state.

The state starts at local `(x=0, y=0, yaw=0)`. It is not an absolute map pose.

`--validate` reads VRS-GPS **after** the EKF run. It selects
`fix_state=4`, matches each reference epoch to the nearest IMU state within
50 ms, and computes 2D ATE RMSE after one rigid SE(2) alignment of the full
trajectory. Alignment adjusts translation and yaw only; it does not adjust
scale. This metric measures trajectory shape against RTK reference. It is not
absolute online position accuracy because the local origin and yaw are
unknown to the filter. VRS measurements never enter EKF prediction or update.

Validation writes `results/validation_summary.txt`,
`results/validation_pairs.csv` and `results/trajectory_validation.png`.
These files are refreshed only by a run with `--validate`; the summary from
the latest run states whether validation was requested.

## Sensor and filter model

The [dataset format](https://sites.google.com/view/complex-urban-dataset/format)
specifies the encoder and IMU column order. The estimator uses gyro z as
rad/s and acceleration x as m/s². `urban35` has an identity vehicle-to-IMU
rotation in `Vehicle2IMU.txt`. Wheel speed uses the dataset's
`EncoderParameter.txt`.

The EKF state is `[x, y, yaw, v, gyro_bias, accel_bias]`. IMU samples drive
prediction; wheel speed drives a scalar update with a 95% NIS gate.
Wheel speed does not constrain absolute yaw or gyro bias, so heading drift is
expected.

## Checks

```bash
python3 -m unittest discover -s tests -v
```

Module boundaries, timing, coordinate frames and extension points are
documented in [docs/architecture.md](docs/architecture.md). The sensor
selection and requirements are in [docs/design_document.md](docs/design_document.md).
