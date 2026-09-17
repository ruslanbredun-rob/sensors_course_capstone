# GPS-denied vehicle localization

Course capstone based on the Complex Urban Dataset, sequence `urban35`. The
planned estimator fuses wheel encoders and Xsens IMU in a 2D EKF. VRS-GPS is
reserved for independent evaluation and is never used as an estimator input.

**Current status:** repository structure and implementation contracts only.
`python main.py` currently stops with an explicit `NotImplementedError`. It
does not yet satisfy homework 18 or 19.

## Dataset

| Source | Approximate size | Download | Local location |
|---|---:|---|---|
| Complex Urban Dataset `urban35` | about 5.5 GB extracted | [official dataset page](https://sites.google.com/view/complex-urban-dataset) | `data/complex_urban/urban35/` |

The dataset is not included in the course submission ZIP. See
[data/README.md](data/README.md) for the expected directories and known missing
frames.

## Planned run

```bash
python -m pip install -r requirements.txt
python main.py
```

The default mode is `base` (homework 18). Other planned modes are `slip`,
`visual`, `full`, and `all` (homework 19). `--dataset` and `--output`
override repository-relative paths from `config/default.json`.

Once implemented, Base will print or save every estimated state. The final
pipeline will print baseline and fused RMSE/ATE in metres and save trajectory
and error plots under `results/`. These are planned outputs, not current
results.

## Project map

| Path | Responsibility |
|---|---|
| `main.py`, `config/` | CLI and explicit run configuration |
| `src/dataset.py`, `src/synchronization.py` | sensor input and timestamps |
| `src/wheel_odometry.py`, `src/ekf.py` | baseline, prediction and updates |
| `src/pipeline.py` | mode orchestration |
| `src/evaluation.py`, `src/visualization.py` | homework 19 metrics and plots |
| `docs/design_document.md` | homework 17 design |
| `docs/architecture.md` | module boundaries, data/control flow and risks |
| `docs/roadmap.md` | implementation order and validation gates |
| `slides/` | defense PDF, prepared after measured results exist |

The immediate implementation target is the wheel + IMU Base pipeline. Column
order, physical units, vehicle axes and VRS frame must be confirmed before
writing the reader and EKF formulas.
