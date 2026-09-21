# GPS-denied vehicle localization

Курсовий проєкт для ДЗ 18–19: planar localization автомобіля на Complex Urban
Dataset, sequences `urban35`, `urban33` та `urban39`. VRS-GPS не входить у
fusion і використовується лише для незалежної оцінки готової траєкторії.

## Реалізовані конфігурації

- **INS baseline:** Xsens IMU prediction + wheel speed і differential
  wheel yaw rate;
- **Baseline + stereo VO:** metric body-frame `dx, dy, dyaw` від stereo camera;
- **Baseline + LiDAR:** scan-to-scan planar ICP з left VLP-16;
- **Selected fusion:** health-gated LiDAR corrections і stereo VO у прогалинах.

EKF state: `[x, y, yaw, speed, gyro_z_bias, accel_x_bias]`. VO/LO передають
body-frame relative pose. EKF порівнює її зі збереженою pose clone та збільшує
measurement covariance за NIS для слабких relative updates.

## Дані

Завантажте sequences із
[Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset) і
розпакуйте в `data/complex_urban/<sequence>/`. Сирі дані не входять у Git.
Структура каталогів описана в [data/README.md](data/README.md).

Для baseline потрібні `encoder.csv`, `xsens_imu.csv`, `EncoderParameter.txt` і
`Vehicle2IMU.txt`. `--validate` також читає `vrs_gps.csv` та
`Vehicle2VRS.txt`. Visual mode потребує stereo images і camera calibration;
LiDAR mode — `VLP_left` scans та `Vehicle2LeftVLP.txt`.

## Середовище

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Запуск

Базовий прототип ДЗ 18:

```bash
python -m src.main --mode base --validate
```

Повний прогін ДЗ 19 для трьох sequences, від короткої до довгої:

```bash
for sequence in urban35 urban33 urban39; do
  python -m src.main \
    --mode all \
    --validate \
    --dataset "data/complex_urban/${sequence}" \
    --output "results/${sequence}"
done
```

Для tuning EKF без повторного обчислення VO/LO додайте
`--reuse-frontends`. Кеші мають бути створені тією самою версією frontend.

Окремий LiDAR experiment без читання camera frames:

```bash
python -m src.main --mode lidar --validate \
  --dataset data/complex_urban/urban35 \
  --output results/urban35
```

## Результати

Primary metric — 2D ATE після rigid SE(2) alignment без scale fit. Initial-pose
metric окремо фіксує старт і початковий напрям, тому показує накопичений drift
без глобальної компенсації yaw. RTK metrics також розбиваються на безперервні
segments, якщо між valid fixes є прогалина понад 1.5 с.

| Sequence | IMU + wheel | +VO | +LiDAR | Selected fusion |
|---|---:|---:|---:|---:|
| `urban35`, global RMSE | **4.153 м** | **4.153 м** | **4.153 м** | **4.153 м** |
| `urban33`, global RMSE | 83.828 м | 87.403 м | 61.992 м | **61.961 м** |
| `urban39`, global RMSE | 188.456 м | 197.715 м | 91.581 м | **91.522 м** |

На `urban35` LiDAR вимикається до запуску frontend, бо turning fraction 1.3% є
нижчою за health threshold 5%. На `urban33/39` він активний. Global alignment
може приховувати accumulated yaw error, тому його треба читати разом з
initial-pose plots. Stereo VO має scale/bias, а scan-to-scan LiDAR накопичує
drift; для вищої точності потрібні VIO/LIO/LVIO або loop closure. Повні метрики,
RTK coverage і обмеження наведені у
[results/conclusions.md](results/conclusions.md). Архітектура описана в
[docs/architecture.md](docs/architecture.md).

Pipeline створює числові CSV і PNG:

- `results/<sequence>/comparison_metrics.csv`;
- `results/<sequence>/rtk_segment_metrics.csv`;
- `results/<sequence>/validation_pairs.csv`;
- `results/<sequence>/estimated_state_<mode>.csv`;
- `results/<sequence>/diagnostics*.csv`;
- `results/<sequence>/screenshots/*.png`.

Статичний файл висновків код не генерує і не перезаписує.

## Перевірка

```bash
python -m unittest discover -s tests -v
```
