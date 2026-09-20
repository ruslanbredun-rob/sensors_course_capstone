# GPS-denied vehicle localization

Курсовий проєкт для ДЗ 18–19: planar localization автомобіля на Complex Urban
Dataset, sequence `urban35`. Система обробляє wheel encoders, Xsens IMU, stereo
camera та left VLP-16. VRS-GPS не входить у fusion і використовується лише для
післяпроцесингової оцінки траєкторії.

## Що реалізовано

- **E1 Base:** IMU prediction + wheel-speed update у 2D EKF;
- **E2 Kinematics + slip:** differential-wheel yaw, gyro-bias correction,
  debounced slip detector і fallback без wheel updates;
- **E3 Visual:** calibrated stereo rectification, ORB/RANSAC visual odometry,
  metric scale зі stereo disparity, quality та NIS gates;
- **E4 Full:** E3 + calibrated VLP-16 preprocessing і planar ICP;
- raw wheel-only comparator, контрольована інжекція wheel fault;
- VRS-GPS evaluation по всій траєкторії, comparison CSV, NIS diagnostics і PNG
  зі скрінами результатів.

EKF state: `[x, y, yaw, speed, gyro_z_bias, accel_x_bias]`. Координати локальні;
початок і початковий yaw довільні.

## Дані

Завантажте `urban35` з
[Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset) і
розпакуйте в `data/complex_urban/urban35/`. Дані займають близько 5.5 GB і не
входять у Git або архів здачі. Детальна структура є в
[data/README.md](data/README.md).

Для E1/E2 потрібні:

```text
data/complex_urban/urban35/
├── calibration/
│   ├── EncoderParameter.txt
│   ├── Vehicle2IMU.txt
│   └── Vehicle2VRS.txt       # лише --validate
└── sensor_data/
    ├── encoder.csv
    ├── xsens_imu.csv
    └── vrs_gps.csv           # лише --validate
```

E3 також потребує `calibration/{left.yaml,right.yaml,Vehicle2Stereo.txt}` і
`image/{stereo_left,stereo_right}/`. E4 потребує
`calibration/Vehicle2LeftVLP.txt` і `sensor_data/VLP_left/`.

## Середовище

Запускайте з кореня репозиторію:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## ДЗ 18: мінімальний прототип

```bash
python main.py --mode base
```

Команда читає IMU та енкодери з різними timestamps, запускає EKF і записує стан
після кожного IMU prediction у `results/estimated_state_base.csv`. Для короткої
перевірки можна додати `--max-events 1000`.

## ДЗ 19: повне порівняння

```bash
python main.py --mode all --validate
```

Повний CPU run займає приблизно дві хвилини в поточному середовищі. Він створює:

- `results/run_summary.txt` і `comparison_metrics.csv`;
- `results/conclusions.md`;
- `results/estimated_state_<mode>.csv` та diagnostics CSV;
- `results/visual_odometry.csv` і `lidar_odometry.csv`;
- `results/screenshots/trajectory_comparison.png`;
- `results/screenshots/error_over_time.png`;
- `results/screenshots/rmse_comparison.png`;
- `results/screenshots/filter_consistency.png`;
- `results/screenshots/metrics_summary.png`.

Контрольована перевірка slip detector:

```bash
python main.py --mode slip --inject-slip --validate \
  --output results/slip_injection
```

Вона додає 25% scale fault правого колеса на інтервалі 60–70 с і створює
`results/slip_injection/screenshots/slip_detection.png`.

Шляхи можна перевизначити через `--dataset PATH`, `--output PATH` або
`config/default.json`.

## Результати `urban35`

Метрика — 2D ATE на 169 valid `VRS fix_state=4` epochs. Локальна траєкторія один
раз вирівнюється з UTM через rigid SE(2): translation і yaw коригуються, scale не
змінюється.

| Конфігурація | RMSE, м | P95, м |
|---|---:|---:|
| Raw wheel odometry | 4.625 | 7.116 |
| E1 INS baseline: IMU + wheel speed | 38.441 | 72.350 |
| E2 kinematics + slip | **3.788** | 5.958 |
| E3 + stereo VO | 3.842 | **5.774** |
| E4 + stereo VO + LiDAR | 3.844 | 5.780 |

E1 є базовою INS конфігурацією: IMU виконує prediction, а wheel speed дає
correction. Raw wheel odometry існує лише як нефільтрований comparator для ДЗ19.
E2 знижує RMSE на 18.1% відносно raw wheel odometry. У контрольованому fault test detector
має 99.8% detection rate і 0.24% false-positive samples; slip-aware EKF дає
3.557 м RMSE проти 113.838 м для пошкодженої raw wheel odometry.

Детальні висновки: [results/conclusions.md](results/conclusions.md). Архітектура,
кадри координат і межі модулів: [docs/architecture.md](docs/architecture.md).

## Перевірка коду

```bash
python -m unittest discover -s tests -v
```
