# GPS-denied vehicle localization

Курсовий проєкт для ДЗ 18–19: planar localization автомобіля на Complex Urban
Dataset, sequence `urban35`. Базова система обробляє wheel encoders та Xsens
IMU з порівнянною частотою близько 100 Hz. Stereo camera і left VLP-16
перевіряються як окремі зовнішні джерела. VRS-GPS не входить у fusion і
використовується лише для післяпроцесингової оцінки траєкторії.

## Що реалізовано

- **E1 Base:** IMU orientation/angular-rate propagation + wheel linear speed і
  differential-wheel course; position інтегрується у 2D EKF;
- **E2 Kinematics + slip:** differential-wheel yaw, gyro-bias correction,
  debounced slip detector і fallback без wheel updates;
- **E2 + Visual:** calibrated stereo rectification, ORB/RANSAC visual odometry,
  metric scale зі stereo disparity, quality та NIS gates;
- **E2 + LiDAR:** окремий режим без камер з calibrated VLP-16 preprocessing і
  planar ICP;
- **E2 + Visual + LiDAR:** обидва зовнішні frontends з guarded fallback;
- контрольована інжекція wheel fault для перевірки slip detector;
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

Режим `visual` потребує
`calibration/{left.yaml,right.yaml,Vehicle2Stereo.txt}` і
`image/{stereo_left,stereo_right}/`. Режим `lidar` працює без камер і потребує
лише `calibration/Vehicle2LeftVLP.txt` та `sensor_data/VLP_left/`. Режим `full`
потребує обидва набори.

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
python -m src.main --mode base
```

Команда читає IMU та енкодери з різними timestamps, запускає EKF і записує стан
після кожного IMU prediction у `results/estimated_state_base.csv`. Для короткої
перевірки можна додати `--max-events 1000`.

## ДЗ 19: повне порівняння

```bash
python -m src.main --mode all --validate
```

Повний CPU run займає приблизно дві хвилини в поточному середовищі. Він створює:

- `results/comparison_metrics.csv` і `validation_pairs.csv`;
- `results/estimated_state_<mode>.csv` та diagnostics CSV;
- `results/visual_odometry.csv` і `lidar_odometry.csv`;
- `results/trajectory_validation.png`;
- `results/screenshots/trajectory_comparison.png`;
- `results/screenshots/error_over_time.png`;
- `results/screenshots/rmse_comparison.png`;
- `results/screenshots/filter_consistency.png`;
- `results/screenshots/metrics_summary.png`.

Текстовий опис і висновки зберігаються у статичному
[`results/conclusions.md`](results/conclusions.md). Pipeline цей файл не генерує
і не перезаписує.

Контрольована перевірка slip detector:

```bash
python -m src.main --mode slip --inject-slip --validate \
  --output results/slip_injection
```

Вона додає 25% scale fault правого колеса на інтервалі 60–70 с і створює
`results/slip_injection/screenshots/slip_detection.png`.

Шляхи можна перевизначити через `--dataset PATH`, `--output PATH` або
`config/default.json`.

Окремий експеримент E2 + LiDAR без читання camera frames:

```bash
python -m src.main --mode lidar --validate
```

## Результати `urban35`

Метрика — 2D ATE на 169 valid `VRS fix_state=4` epochs уздовж траєкторії
3215 м. Локальна траєкторія один раз вирівнюється з UTM через rigid SE(2):
translation і yaw коригуються, scale не змінюється.

| Конфігурація | RMSE, м | P95, м |
|---|---:|---:|
| E1 INS baseline: IMU + wheel | 4.153 | 7.024 |
| E2 kinematics + slip | **3.788** | 5.958 |
| E2 + stereo VO | **3.788** | **5.953** |
| E2 + LiDAR, no camera | 3.788 | 5.958 |
| E2 + stereo VO + LiDAR | **3.788** | **5.953** |

E1 є базовою INS конфігурацією: IMU дає angular rate для orientation, а колеса —
лінійну швидкість та незалежний course increment. Position інтегрується в EKF
із цих двох джерел. IMU-only конфігурації немає, бо подвійне інтегрування
acceleration не дає стійкої лінійної швидкості. E2 знижує RMSE на 8.8%
відносно E1. Зовнішні frontends використовуються тільки під час деградації wheel
updates, тому LiDAR-only режим зберігає результат E2, а один прийнятий VO
increment змінює RMSE лише на 0.00004 м. У контрольованому fault test detector
має 99.8% detection rate і 0.24% false-positive samples; slip-aware EKF дає
3.557 м RMSE.

Саме `urban35` є легкою послідовністю для Wheel+IMU: рух переважно плавний, а
slip detector активний лише для 20 із 17 387 wheel samples. Тому VO та LiDAR
майже нічого виправляти. Реалізовані stereo VO й ICP є окремими frontends, а не
повноцінними VIO/LIO зі спільною оптимізацією та IMU deskew.

Детальні висновки: [results/conclusions.md](results/conclusions.md). Архітектура,
кадри координат і межі модулів: [docs/architecture.md](docs/architecture.md).

## Перевірка коду

```bash
python -m unittest discover -s tests -v
```
