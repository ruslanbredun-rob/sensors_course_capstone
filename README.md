# GPS-denied vehicle localization

Курсовий проєкт для ДЗ 18–19: planar localization автомобіля на Complex Urban
Dataset, sequences `urban33`, `urban35` та `urban39`. VRS-GPS не входить у fusion і
використовується лише для незалежної оцінки готової траєкторії.

## Реалізовані конфігурації

- **E1 Base:** Xsens IMU prediction + wheel speed і differential-wheel course;
- **E2 Kinematics + slip:** E1 + wheel/IMU disagreement detector;
- **E2 + Visual:** E2 + calibrated metric stereo VO;
- **E2 + LiDAR:** E2 + calibrated left VLP-16 planar ICP без camera data;
- **E2 + Visual + LiDAR:** continuous LiDAR update та VO для прогалин LiDAR.

EKF state: `[x, y, yaw, speed, gyro_z_bias, accel_x_bias]`. VO/LO передають
body-frame `dx, dy, dyaw`. EKF зіставляє increment зі збереженою попередньою
pose clone та її covariance, формує innovation по `x, y, yaw` і адаптивно
збільшує measurement covariance за NIS.

## Дані

Завантажте `urban33`, `urban35` та `urban39` з
[Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset) і
розпакуйте в `data/complex_urban/<sequence>/`. Сирі дані не входять у Git.
Структура каталогів описана в [data/README.md](data/README.md).

Для E1/E2 потрібні `encoder.csv`, `xsens_imu.csv`, `EncoderParameter.txt` та
`Vehicle2IMU.txt`. `--validate` також читає `vrs_gps.csv` і
`Vehicle2VRS.txt`. Visual mode потребує stereo images і camera calibration;
LiDAR mode — `VLP_left` scans і `Vehicle2LeftVLP.txt`.

## Середовище

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Запуск

ДЗ 18, базовий прототип:

```bash
python -m src.main --mode base
```

Повне порівняння `urban35`:

```bash
python -m src.main --mode all --validate
```

Повне порівняння `urban39`:

```bash
python -m src.main --mode all --validate \
  --dataset data/complex_urban/urban39 \
  --output results/urban39
```

Розраховані VO/LO можна повторно використати для швидкого tuning EKF:

```bash
python -m src.main --mode all --validate --reuse-frontends \
  --dataset data/complex_urban/urban39 \
  --output results/urban39
```

`--reuse-frontends` читає `visual_odometry.csv` і `lidar_odometry.csv` з
output directory. Повний raw `urban39` frontend run займає близько 30 хвилин;
повторний filter-only run — близько двох хвилин.

Окремий LiDAR experiment без читання camera frames:

```bash
python -m src.main --mode lidar --validate
```

Контрольована перевірка slip detector:

```bash
python -m src.main --mode slip --inject-slip --validate \
  --output results/urban35/slip_injection
```

## Результати

Primary metric — 2D ATE після rigid SE(2) alignment без scale fit.

| Sequence | E1 IMU+wheel | E2 | E2+VO | E2+LiDAR | Full |
|---|---:|---:|---:|---:|---:|
| `urban33`, RMSE м | 83.828 | 84.267 | 87.574 | 47.475 | **45.763** |
| `urban35`, RMSE м | 4.153 | **3.788** | 3.788 | 3.788 | 3.788 |
| `urban39`, RMSE м | 188.456 | 188.471 | 197.477 | 90.789 | **90.690** |

На `urban35` coverage VO та LO нижче 30%, тому health gate не подає
розріджені increments у EKF. На `urban39` LiDAR coverage становить 97.4% і
continuous relative-pose fusion зменшує RMSE на 51.9%. Stereo VO coverage
44.0%, але його scale/bias у цій простій реалізації погіршує окремий visual run.

Графіки trajectory comparison вирівняні за початковою pose: перші точки
збігаються, а yaw визначається за першим надійним відрізком руху близько 20 м.
Числові таблиці містять standard global ATE та initial-pose error. Деталі та
обмеження: [results/conclusions.md](results/conclusions.md).
Архітектура: [docs/architecture.md](docs/architecture.md).

Pipeline створює тільки числові CSV і PNG:

- `results/<sequence>/comparison_metrics.csv`;
- `results/<sequence>/validation_pairs.csv`;
- `results/<sequence>/estimated_state_<mode>.csv`;
- `results/<sequence>/diagnostics*.csv`;
- `results/<sequence>/screenshots/*.png`.

Статичний файл висновків код не генерує і не перезаписує.

## Перевірка

```bash
python -m unittest discover -s tests -v
```
