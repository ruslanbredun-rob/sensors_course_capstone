# Висновки ДЗ 18–19

## Результати на `urban35`

| Конфігурація | RMSE, м | Median, м | P95, м |
|---|---:|---:|---:|
| E2 kinematics + slip | 3.557 | 2.840 | 6.552 |

Найкращий режим — `slip`: 3.557 м RMSE.
RMSE пораховано по 169 VRS epochs уздовж траєкторії 3215 м.

## Наш шлях обробки даних

1. `encoder.csv` і `xsens_imu.csv` читаються потоково з перевіркою кількості колонок, SI units і строго зростаючих nanosecond timestamps. Обидва потоки мають близько 100 Hz, тому жоден із них не є низькочастотною зовнішньою поправкою.
2. Encoder counts через resolution, діаметри коліс і фактичний `dt` перетворюються на left/right speed, лінійну швидкість та differential-drive course constraint; `x,y` інтегрує EKF.
3. IMU gyro `z` і acceleration `x` виконують high-rate EKF prediction: кутова швидкість поширює orientation/yaw, прискорення — speed. IMU-only trajectory не використовується, бо інтегрування acceleration без надійної початкової лінійної швидкості швидко накопичує drift.
4. E1 Base завжди використовує обидва комплементарні джерела: Wheel + IMU. Wheel update коригує speed і gyro bias; EKF записує `x, y, yaw, speed` після кожного IMU step.
5. E2 додає wheel/IMU disagreement detector. Під час slip wheel correction пропускається, а IMU prediction продовжується.
6. Stereo frontend ректифікує пари, знаходить ORB matches, виконує RANSAC essential matrix і відновлює metric scale зі disparity. LiDAR frontend переводить VLP-16 points у vehicle frame, voxelizes їх та оцінює increment через 2D ICP.
7. Health manager використовує VO лише у degraded wheel interval. LiDAR є другим fallback, якщо немає недавньої якісної VO correction. Низька quality або NIS вище gate залишають стан попереднього етапу.
8. `vrs_gps.csv` не читається estimator-ом. Після завершення run valid RTK epochs зіставляються за часом, траєкторії один раз вирівнюються rigid SE(2) без scale fit, після чого рахується ATE.

## Консистентність і межі

Для `slip` wheel-speed NIS нижче порога 3.841 у 99.97% з 16349 перевірених updates; wheel-yaw NIS нижче порога 6.635 у 99.79% з 16349 updates. Поточна модель шуму консервативна.

VRS-GPS не надходить у EKF. Він використаний після оцінювання стану для часових пар, одного SE(2) вирівнювання без зміни масштабу та ATE по всій траєкторії.

## Контрольована перевірка slip detector

Detection rate: 99.8%; false-positive samples: 0.24%. Під час fault interval wheel updates відкидаються, а EKF продовжує predict за IMU.
