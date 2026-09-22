# Vehicle localization with IMU, wheels, stereo VIO and GPS

Курсовий проєкт для ДЗ 18–19 з Sensor Engineering. Система оцінює плоску
траєкторію автомобіля на послідовностях `urban35`, `urban33` та `urban39` з
Complex Urban Dataset.

## Експерименти

| Mode | Сенсори та призначення |
|---|---|
| `gps` | GPS + IMU + wheels; усі commercial GPS measurements |
| `base` | IMU + wheels; базова GPS-denied одометрія |
| `visual` | IMU + wheels + loosely coupled stereo VO |
| `vio` | IMU + wheels + feature-level stereo VIO |
| `gps_dropout` | IMU + wheels + VIO; GPS відсутній на 20–40% та 50–70% шляху |
| `gps_sparse` | IMU + wheels + VIO; одне GPS measurement кожні 30 с |
| `all` | Послідовний запуск усіх шести режимів |

Commercial GPS із `gps.csv` входить у GPS режими. Точний VRS-GPS ніколи не
входить у estimator і використовується лише як ground truth.

## Встановлення

Потрібен Python 3.10 або новіший.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Дані

Завантажте `urban33`, `urban35` та `urban39` зі сторінки
[Complex Urban Dataset — Download (LiDAR + Stereo)](https://sites.google.com/view/complex-urban-dataset/download-lidar-stereo).
Для роботи потрібні calibration, navigation data та stereo images. Архіви
датасету не входять у репозиторій і submission archive.

Якщо отримано Google Drive folder URL, завантаження можна виконати так:

```bash
python -m pip install gdown
gdown --folder "GOOGLE_DRIVE_FOLDER_URL" \
  -O data/complex_urban/urban39_archives
```

Розпакування однієї послідовності:

```bash
mkdir -p data/complex_urban/urban39
for archive in data/complex_urban/urban39_archives/*.tar.gz; do
  tar -xzf "$archive" -C data/complex_urban/urban39
done
```

Після розпакування мінімальна структура має містити:

```text
data/complex_urban/urban39/
├── calibration/
│   ├── EncoderParameter.txt
│   ├── Vehicle2GPS.txt
│   ├── Vehicle2IMU.txt
│   ├── Vehicle2Stereo.txt
│   ├── left.yaml
│   └── right.yaml
├── image/
│   ├── stereo_left/
│   └── stereo_right/
└── sensor_data/
    ├── encoder.csv
    ├── gps.csv
    ├── vrs_gps.csv
    └── xsens_imu.csv
```

Детальніше: [data/README.md](data/README.md).

Приблизний розмір уже розпакованих даних:

| Sequence | Розмір на диску |
|---|---:|
| `urban35` | 5.5 GB |
| `urban33` | 44 GB |
| `urban39` | 64 GB |
| **Разом** | **≈114 GB** |

Розмір може трохи відрізнятися залежно від набору завантажених архівів.

## Запуск

Один режим:

```bash
python -m src.main --mode vio --validate \
  --dataset data/complex_urban/urban33 \
  --output results/urban33
```

Повний експеримент для трьох послідовностей:

```bash
for sequence in urban35 urban33 urban39; do
  python -m src.main --mode all --validate \
    --dataset "data/complex_urban/${sequence}" \
    --output "results/${sequence}"
done
```

Повторний запуск із готовими сумісними VO/VIO кешами:

```bash
for sequence in urban35 urban33 urban39; do
  python -m src.main --mode all --validate --reuse-frontends \
    --dataset "data/complex_urban/${sequence}" \
    --output "results/${sequence}"
done
```

## Коротка теорія

### EKF

Planar EKF оцінює стан

```text
[x, y, yaw, speed, gyro_z_bias, accel_x_bias]
```

IMU виконує prediction між timestamps. Wheel speed коригує лінійну швидкість,
а differential wheel yaw rate допомагає оцінювати курс і gyro bias. GPS дає
глобальний position update після перетворення WGS84 у UTM, вирівнювання
локальної системи координат і компенсації antenna lever arm.

### VO та VIO

Stereo VO знаходить ORB correspondences, відновлює metric depth із disparity та
оцінює relative pose. VIO використовує ті самі feature observations у
sliding-window optimization разом з IMU і wheel preintegration. Для кожного
camera epoch оптимізуються `[x, y, yaw, speed]`, а gyro та accelerometer biases
є спільними змінними активного вікна.

У `gps_dropout` і `gps_sparse` сусідні VIO states перетворюються на body-frame
`dx, dy, dyaw` factors для EKF. GPS position innovation через Kalman
cross-covariance може коригувати pose, velocity та IMU biases.

### NIS

Normalized Innovation Squared перевіряє узгодженість measurement із поточним
станом та covariance. Для помірного innovation measurement covariance
збільшується адаптивно. Сильний outlier відкидається. Після дуже довгого GPS
outage це може завадити повторній локалізації, якщо накопичений drift перевищив
gate.

## Алгоритм

1. Прочитати calibration та timestamped sensor streams.
2. Перетворити encoder counts у left/right speed, forward speed і yaw rate.
3. Виконувати IMU prediction та wheel corrections у часовому порядку.
4. Для stereo frames виконати rectification, disparity, ORB matching і PnP
   outlier rejection.
5. Побудувати IMU, wheel і camera residuals та оптимізувати активне VIO window.
6. У degraded GPS режимах подати VIO increments і доступні GPS positions у EKF.
7. Порівняти готову траєкторію з valid VRS-GPS fixes та створити CSV і PNG
   artifacts.

## Метрики

- **Global SE(2) ATE** підбирає один оптимальний поворот і зміщення для всієї
  траєкторії. Він добре порівнює форму шляху, але частково приховує початкову
  помилку курсу.
- **Initial-pose RMSE** суміщає лише стартову точку та початковий напрям. Тому
  показує накопичення position і yaw drift від старту.

Низький Global ATE разом із високим Initial-pose RMSE означає, що форма шляху
схожа на reference, але траєкторія має помилку початкового напряму або
накопичений yaw drift.

На trajectory графіках пунктирна VRS-GPS RTK лінія є реальною reference
траєкторією. Кольорова лінія є оцінкою estimator після initial-pose alignment.
Для estimate задаються лише спільна стартова точка та початковий напрям за
першим стабільним відрізком руху. Подальша reference траєкторія не
використовується для підлаштування estimate, а масштаб не змінюється. Тому
графік показує накопичений position і yaw drift, а число в легенді є
`Initial-pose RMSE`.

Global SE(2) ATE залишається у таблицях і на RMSE bar chart як додаткова оцінка
схожості форми після offline суміщення всієї траєкторії.

## Результати

Числові таблиці, три trajectory comparisons для кожної послідовності та
висновки наведені у [results/conclusions.md](results/conclusions.md).

Основні artifacts:

- `comparison_metrics.csv` і `rtk_segment_metrics.csv`;
- `screenshots/rmse_comparison.png`;
- `screenshots/trajectory_gps_reference.png`;
- `screenshots/trajectory_vio.png`;
- `screenshots/trajectory_degraded_gps.png`;
- `screenshots/filter_consistency.png` з NIS diagnostics;
- `results/screenshots/console_rmse_summary.png` з підсумковим RMSE/ATE.

Архітектура описана у [docs/architecture.md](docs/architecture.md) та
[docs/vio_architecture.md](docs/vio_architecture.md).

## Перевірка

```bash
python -m unittest discover -s tests -v
```
