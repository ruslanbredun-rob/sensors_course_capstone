# Source modules

`dataset.py` owns parsing and unit conversion; `synchronization.py` orders
sensor events; `wheel_odometry.py` creates wheel measurements and a raw
baseline; `ekf.py` owns estimator state and covariance; `pipeline.py`
orchestrates them. `evaluation.py` may read VRS reference but cannot feed it
to the estimator. `visualization.py` writes result plots.

All current algorithms are explicit stubs. See [architecture](../docs/architecture.md)
and [implementation plan](../docs/roadmap.md).
