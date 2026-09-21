# GPS-denied vehicle localization

Курсовий проєкт для ДЗ 18–19 на Complex Urban Dataset. У гілці `feature/vio`
реалізовано три незалежні експерименти:

- `base`: planar INS на IMU та колісній кінематиці;
- `visual`: baseline з relative stereo VO corrections;
- `vio`: feature-level stereo VIO без коліс.

VRS-GPS не входить в estimator і використовується лише для оцінки готової
траєкторії.

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

Порівняння baseline, VO та VIO:

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
містить dataset path, image range та параметри estimator. Несумісний cache
відхиляється замість тихого використання.

## Оцінка

Primary metric — 2D ATE після rigid SE(2) alignment без scale fit. Initial-pose
metric фіксує спільну стартову точку та початковий напрям і краще показує
накопичений yaw drift. Окремо формуються метрики для безперервних RTK segments.

Pipeline створює:

- `comparison_metrics.csv` і `rtk_segment_metrics.csv`;
- `estimated_state_<mode>.csv`;
- `vio_trajectory.csv` та `vio_manifest.json`;
- `screenshots/trajectory_comparison.png` та інші графіки.

Поточні VIO результати треба отримати новим прогоном: старий motion-factor VIO cache
несумісний із feature-level estimator. Архітектура описана у
[docs/vio_architecture.md](docs/vio_architecture.md).

## Перевірка

```bash
python -m unittest discover -s tests -v
```
