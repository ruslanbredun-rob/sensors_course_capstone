# Висновки ДЗ 18–19

## Результати на `urban35`

| Конфігурація | RMSE, м | Median, м | P95, м |
|---|---:|---:|---:|
| E1 INS baseline (IMU + wheel) | 4.153 | 3.104 | 7.024 |
| E2 kinematics + slip | 3.788 | 3.259 | 5.958 |
| E2 + stereo VO | 3.788 | 3.262 | 5.953 |
| E2 + LiDAR (no camera) | 3.788 | 3.259 | 5.958 |
| E2 + stereo VO + LiDAR | 3.788 | 3.262 | 5.953 |

Найкращий режим — `visual`: 3.788 м RMSE. Покращення відносно E1 Wheel+IMU становить 8.8%.
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

Для `full` wheel-speed NIS нижче порога 3.841 у 99.98% з 17367 перевірених updates; wheel-yaw NIS нижче порога 6.635 у 99.79% з 17367 updates. Поточна модель шуму консервативна.

VRS-GPS не надходить у EKF. Він використаний після оцінювання стану для часових пар, одного SE(2) вирівнювання без зміни масштабу та ATE по всій траєкторії.

Strict health gating не дозволив VO або LiDAR погіршити E2. На звичайній послідовності використано VO: 1/184 corrections; LiDAR без камер: 0/27 corrections. Frontends повністю обробили дані, але estimator приймав correction лише під час degraded wheel interval.

## Чому VO та LiDAR майже не покращили результат

Саме `urban35` є легкою послідовністю для Wheel+IMU: обидва потоки працюють приблизно зі 100 Hz, рух переважно плавний, а slip detector був активний лише для 20 із 17387 wheel samples (0.12%). Тому E2 вже добре відтворює форму траєкторії, а зовнішнім сенсорам майже нічого виправляти.

Реалізовані frontends не є повноцінними VIO/LIO. Stereo VO не оптимізує features разом з IMU state, а LiDAR ICP не робить IMU deskew rolling scan. При постійному fusion їхні noisy increments трохи погіршували RMSE, тому health manager використовує їх лише як fallback. На цій послідовності це означає практично однаковий результат E2, E2+VO та E2+LiDAR.

## Порівняння з VIO та LIO

Поточна система є **loosely coupled**: stereo VO та LiDAR ICP спочатку окремо оцінюють relative motion, після чого EKF отримує лише speed/yaw-rate correction. Cross-covariance features, point clouds, IMU bias і state при цьому втрачається.

**VIO** спільно оптимізує camera reprojection residuals, IMU preintegration, pose, velocity і biases. Це краще утримує scale та orientation, але потребує точної camera–IMU calibration, ініціалізації й складнішого nonlinear solver.

**LIO** використовує IMU для deskew кожного LiDAR scan і спільно оцінює trajectory та scan residuals. Це прямо усуває основну проблему нашого VLP-16 ICP — rolling motion distortion. Ціна — точна time/extrinsic calibration, більший state і суттєво більше обчислень.
