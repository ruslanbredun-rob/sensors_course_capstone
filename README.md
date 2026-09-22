# GPS-denied vehicle localization

Курсовий проєкт для ДЗ 18–19 на Complex Urban Dataset. Реалізовано шість
незалежних експериментів:

- `gps`: reference configuration — IMU + колеса + всі commercial GPS measurements;
- `base`: planar INS на IMU та колісній кінематиці;
- `visual`: IMU + колеса + relative stereo VO corrections;
- `vio`: IMU + колеса + feature-level stereo VIO;
- `gps_dropout`: IMU + колеса + VIO, GPS відсутній на 20–40% та 50–70% шляху;
- `gps_sparse`: IMU + колеса + VIO та одне commercial GPS measurement кожні 30 с.

Commercial `gps.csv` входить лише у три GPS режими. Назва reference
configuration означає повний доступ до commercial GPS. VRS-GPS ніколи не
входить в estimator і використовується лише як ground truth для оцінки.

## Середовище

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Дані розпаковуються у `data/complex_urban/<sequence>/`. Для `base` потрібні IMU,
encoders та їх calibration. Для `visual` і `vio` також потрібні stereo images,
`left.yaml`, `right.yaml` та `Vehicle2Stereo.txt`.

## Запуск

Окремий VIO:

```bash
python -m src.main --mode vio --validate \
  --dataset data/complex_urban/urban33 \
  --output results/urban33
```

Порівняння всіх шести режимів:

```bash
for sequence in urban35 urban33 urban39; do
  python -m src.main --mode all --validate \
    --dataset "data/complex_urban/${sequence}" \
    --output "results/${sequence}"
done
```

Повторна побудова графіків із сумісних кешів:

```bash
python -m src.main --mode all --validate --reuse-frontends \
  --dataset data/complex_urban/urban33 \
  --output results/urban33
```

VIO cache складається з `vio_trajectory.csv` і `vio_manifest.json`. Manifest
містить dataset path, image range, IMU/wheel sources та параметри estimator. Несумісний cache
відхиляється замість тихого використання.

## Оцінка

Primary metric — 2D ATE після rigid SE(2) alignment без scale fit. Initial-pose
metric фіксує спільну стартову точку та початковий напрям і краще показує
накопичений yaw drift. Окремо формуються метрики для безперервних RTK segments.

Pipeline створює:

- `comparison_metrics.csv` і `rtk_segment_metrics.csv`;
- `estimated_state_<mode>.csv`;
- `vio_trajectory.csv` та `vio_manifest.json`;
- `screenshots/trajectory_gps_reference.png`, `trajectory_vio.png` і
  `trajectory_degraded_gps.png`;
- error, RMSE та diagnostics графіки.

Поточні VIO результати треба отримати новим прогоном: старий motion-factor VIO cache
несумісний із feature-level estimator. Архітектура описана у
[docs/vio_architecture.md](docs/vio_architecture.md).

## Перевірка

```bash
python -m unittest discover -s tests -v
```
