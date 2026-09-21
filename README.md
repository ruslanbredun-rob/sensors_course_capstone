# GPS-denied vehicle localization

Курсовий проєкт для ДЗ 18–19: planar localization автомобіля на Complex Urban
Dataset, sequences `urban35`, `urban33` та `urban39`. VRS-GPS не входить у
fusion і використовується лише для незалежної оцінки готової траєкторії.

## Реалізовані конфігурації

- **Adaptive INS baseline:** Xsens IMU prediction + wheel speed і differential
  wheel yaw rate;
- **Baseline + stereo VO:** metric body-frame `dx, dy, dyaw` від stereo camera;
- **Baseline + LiDAR:** IMU-deskewed hybrid scan-to-scan/scan-to-local-map
  odometry з left VLP-16;
- **Full:** LiDAR corrections і stereo VO у прогалинах LiDAR.

EKF state: `[x, y, yaw, speed, gyro_z_bias, accel_x_bias]`. Колісні noise та
gyro-bias random walk адаптуються окремо для прямого руху і поворотів. VO/LO
передають body-frame relative pose. EKF порівнює її зі збереженою pose clone і
адаптивно збільшує measurement covariance за NIS.

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

| Sequence | Adaptive baseline | +VO | +LiDAR | Full |
|---|---:|---:|---:|---:|
| `urban35`, RMSE м | 3.978 | 3.978 | **3.259** | **3.259** |
| `urban33`, RMSE м | 68.508 | 72.190 | **67.932** | 68.298 |
| `urban39`, RMSE м | 180.640 | 190.904 | **176.724** | 176.730 |

`urban35` є коротким майже прямим маршрутом, де baseline вже добре відтворює
форму. На довгих sequences прості VO/LO frontends не усувають систематичний yaw
drift: local-map correction допомагає мало, а stereo VO може погіршувати оцінку
через scale/bias. Деталі, RTK coverage і обмеження наведені у
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
