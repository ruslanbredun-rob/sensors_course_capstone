# Feature-level planar stereo VIO

## Мета

Estimator будує окрему GPS-denied траєкторію зі stereo camera та IMU. Колеса не
використовуються. Це усуває повторне використання IMU, яке було у попередній
схемі `motion-factor VIO -> EKF`.

## State і factors

Для кожного camera epoch оптимізується:

```text
[x, y, yaw, forward_speed]
```

На все активне вікно спільно оцінюються `gyro_z_bias` та `accel_x_bias`.

Між сусідніми станами діють два типи factors:

1. **IMU preintegration** — інтегрує gyro Z та acceleration X точно між camera
   timestamps і задає residuals для position, yaw та speed.
2. **Stereo reprojection** — disparity попереднього stereo pair дає metric 3D
   points; ORB matches дають 2D observations у поточному left image.
   Reprojection залежить від двох vehicle poses та camera extrinsic.

PnP RANSAC використовується для outlier rejection та початкового наближення,
але фінальний рух визначає спільна nonlinear optimization. `least_squares`
використовує Huber loss.

## Runtime

```text
stereo timestamps
  -> rectification + disparity + ORB
  -> match pair and reject outliers by PnP RANSAC
  -> preintegrate IMU on the same interval
  -> append pose/speed state and factors
  -> optimize active window
  -> emit newest state
```

Якщо visual factor для interval не пройшов gate, camera epoch не губиться: VIO
додає IMU-only edge. Це зберігає неперервність timestamps і preintegration.

## Cache

`vio_trajectory.csv` містить states. `vio_manifest.json` містить schema version,
absolute dataset path, image timestamps, image scale, frame step, VIO config та
frontend statistics. `--reuse-frontends` приймає cache лише за точного збігу
manifest. Зміна sequence або tuning параметрів вимагає нового прогону.

## Обмеження

- planar motion: немає roll, pitch, vertical velocity та gravity alignment;
- mean gyro/acceleration preintegration без повної covariance propagation;
- landmarks живуть у pair factor, довгі multi-frame feature tracks відсутні;
- fixed-lag anchor замість повної marginalization;
- немає loop closure, тому довготривалий drift лишається.

Отже це повноцінне поєднання feature reprojection та IMU factors у межах
planar моделі курсового проєкту, але не production 3D VINS.

## Запуск

```bash
python -m src.main --mode vio --validate \
  --dataset data/complex_urban/urban33 \
  --output results/urban33
```

Після першого прогону:

```bash
python -m src.main --mode vio --validate --reuse-frontends \
  --dataset data/complex_urban/urban33 \
  --output results/urban33
```
