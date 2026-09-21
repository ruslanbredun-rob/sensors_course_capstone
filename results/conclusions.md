# Результати та висновки ДЗ 18–19

## Умови порівняння

VRS-GPS не надходить у EKF. Для оцінки беруться тільки `fix_state=4`, VRS
epochs зіставляються зі state за timestamp у межах 50 мс.

Primary metric — global rigid SE(2) ATE без scale fit. Окремо рахується
initial-pose error: translation суміщає перші точки, а yaw оцінюється за першим
надійним відрізком руху близько 20 м. На trajectory plots початкова позиція і
напрям estimate та reference збігаються.

| Характеристика | `urban33` | `urban35` | `urban39` |
|---|---:|---:|---:|
| Тривалість | 1284.4 с | 173.9 с | 1866.8 с |
| Encoder samples | 128 436 | 17 388 | 186 675 |
| IMU samples | 128 442 | 17 388 | 186 682 |
| Валідні RTK epochs | 908 | 169 | 314 |
| Polyline по valid RTK epochs | 7162 м | 3215 м | 5577 м |
| Wheel distance | ≈7.4 км | ≈3.2 км | ≈10.7 км |

`urban35` — короткий майже прямий маршрут. `urban33` та `urban39` містять
багато поворотів і краще показують вплив yaw drift та odometry corrections.

## Результати `urban33`

| Конфігурація | Global RMSE, м | Median, м | P95, м | Initial-pose RMSE, м |
|---|---:|---:|---:|---:|
| E1 IMU + wheel | 83.828 | 41.393 | 148.782 | 119.094 |
| E2 kinematics + slip | 84.267 | 41.894 | 150.274 | 119.373 |
| E2 + stereo VO | 87.574 | 40.944 | 167.555 | 160.326 |
| E2 + LiDAR | 47.475 | **26.631** | 87.441 | **110.466** |
| E2 + stereo VO + LiDAR | **45.763** | 28.029 | **84.360** | 118.525 |

![Траєкторії urban33](urban33/screenshots/trajectory_comparison.png)

Stereo frontend прийняв 3117/6410 пар (48.6%), LiDAR — 12210/12732 (95.9%).
Full mode знижує global RMSE на 45.4%. За initial-pose alignment LiDAR-only
має найменший RMSE, а full mode майже дорівнює baseline. Водночас кінцева
помилка зменшується з 404.1 м для baseline до 31.5 м для LiDAR і 30.4 м для
full. Це означає, що LO добре стримує довготривалий yaw drift, але локальна
форма середньої частини маршруту ще має систематичну похибку.

## Результати `urban35`

| Конфігурація | Global RMSE, м | Median, м | P95, м | Initial-pose RMSE, м |
|---|---:|---:|---:|---:|
| E1 IMU + wheel | 4.153 | 3.104 | 7.024 | 52.034 |
| E2 kinematics + slip | **3.788** | 3.259 | 5.958 | **51.433** |
| E2 + stereo VO | 3.788 | 3.259 | 5.958 | 51.433 |
| E2 + LiDAR | 3.788 | 3.259 | 5.958 | 51.433 |
| E2 + stereo VO + LiDAR | 3.788 | 3.259 | 5.958 | 51.433 |

![Траєкторії urban35](urban35/screenshots/trajectory_comparison.png)

E2 знижує global RMSE на 8.8%. Stereo frontend прийняв 184/868 пар, LiDAR —
27/1723. Coverage становить 21.2% та 1.6%, нижче health threshold 30%, тому
розріджені relative transforms не подаються у EKF. Через це VO/LO modes
зберігають результат E2. На майже прямому маршруті це очікувано: Wheel+IMU вже
добре відтворює форму траєкторії, а frontends не дають безперервного constraint.

## Результати `urban39`

| Конфігурація | Global RMSE, м | Median, м | P95, м | Initial-pose RMSE, м |
|---|---:|---:|---:|---:|
| E1 IMU + wheel | 188.456 | 108.539 | 348.793 | 397.401 |
| E2 kinematics + slip | 188.471 | 108.484 | 348.879 | 397.457 |
| E2 + stereo VO | 197.477 | 110.616 | 367.559 | 412.119 |
| E2 + LiDAR | 90.789 | 57.316 | 168.823 | **247.936** |
| E2 + stereo VO + LiDAR | **90.690** | **57.029** | **168.689** | 249.365 |

![Траєкторії urban39](urban39/screenshots/trajectory_comparison.png)

LiDAR-only зменшує global RMSE на 51.8%, full mode — на 51.9%. LiDAR
frontend прийняв 18022/18506 increments, або 97.4%, тому утворює continuous
odometry constraint. Full mode застосував 18143 із 22121 доступних increments:
LiDAR має пріоритет, а VO заповнює його прогалини.

Stereo VO прийняв 4099/9323 пар, але окремий visual run погіршив RMSE на 4.8%.
Причина — simple essential-matrix/stereo-scale frontend має більший translation
scatter і систематичний bias. Adaptive NIS обмежує окремі outliers, але не може
усунути bias послідовності. Full mode не виконує подвійний update від VO та LO
на тому самому інтервалі, бо ці вимірювання корельовані.

## Як обробляються дані

1. Readers потоково читають headerless CSV і перевіряють кількість полів,
   timestamps, SI ranges та calibration.
2. Encoder cumulative counts диференціюються за фактичним `dt`. Діаметри коліс,
   resolution та wheelbase переводять їх у signed left/right speed, forward
   speed і differential yaw rate.
3. IMU gyro z та acceleration x виконують EKF prediction. Wheel measurement
   коригує speed і gyro bias.
4. Slip detector порівнює wheel yaw/acceleration з IMU, має debounce і під час
   active state пропускає wheel updates.
5. Stereo VO повертає metric body-frame `dx,dy,dyaw` після rectification,
   ORB/RANSAC, disparity scale та повного camera-to-vehicle transform.
6. LiDAR points переводяться у vehicle frame, фільтруються, voxelized; planar
   ICP оцінює body-frame `dx,dy,dyaw`.
7. На попередньому frontend epoch EKF зберігає pose clone, covariance та
   cross-covariance. Relative pose innovation коригує `x,y,yaw`.
8. NIS вище soft threshold збільшує measurement covariance. Лише outlier, який
   потребує scale понад 100, повністю відкидається.
9. Після estimator run VRS використовується для global та initial-pose
   evaluation. Він не впливає на state.

## Перевірка причин Urban39 error

### Wheelbase і wheel kinematics

Calibration wheelbase дорівнює `1.52439 м`. Після усереднення на 0.5–2 с
регресія IMU/FOG yaw до wheel yaw дає scale близько 1.0, а effective wheelbase
близько 1.53 м. Знак yaw правильний. Отже помилка не пояснюється неправильним
wheelbase або local-to-global handedness.

### Gyro bias і повороти

FOG audit показує, що форма коротких поворотів збігається, але на всьому
`urban39` wheel/IMU yaw має малий довготривалий offset. За 31 хвилину різниця
накопичується приблизно до 1.8 рад. На `urban35` такий offset майже непомітний;
на петльовій траєкторії він створює сотні метрів position drift.

### Slip

Без synthetic fault detector активний лише для 40 із 186674 wheel samples
(0.02%). Тому natural slip у поточному detector не є головною причиною великої
помилки. Thresholds лишаються консервативними через шум differentiation
encoder speed на 100 Hz.

### Extrinsics і transforms

Camera та LiDAR rotations ортонормальні, determinant близький до 1. LiDAR points
переводяться до vehicle origin до ICP. Для camera translation додано lever-arm
term `t - R*t`, якого не було в першій реалізації. Relative transform не
додається як global `dx,dy`: він порівнюється з body-frame delta від anchor
pose.

### Covariance та NIS

Початковий LiDAR `yaw_std=0.3 rad` робив LO практично неактивним. Після
калібрування за scan-to-scan residuals uncertainty становить приблизно
0.08–0.10 м для translation і 0.003–0.006 рад для yaw залежно від quality.
Adaptive NIS збільшує недовіру до слабкого measurement, а не одразу відкидає
його. На фінальному Urban39 LiDAR run covariance було збільшено для 398 із
18022 increments; hard rejection не знадобився.

## Вирівнювання старту та різниця метрик

Попередній plot показував global Kabsch alignment. Він мінімізує сумарну ATE і
може змістити першу точку; на старому Urban39 графіку цей offset становив
приблизно 349 м. Це була властивість візуалізації, а не timestamp shift.

Перші приблизно 6 секунд `urban39` стоїть, тому напрям за першими двома RTK
точками визначається шумом. Тепер trajectory plots суміщають стартову позицію,
чекають на перший надійний відрізок близько 20 м і за ним суміщають початковий
yaw.

Initial-pose RMSE вищий за global ATE, бо global Kabsch fit підбирає кут і зсув
за всією траєкторією та мінімізує середню помилку. Initial-pose alignment фіксує
напрям на старті, після чого accumulated yaw drift уже не компенсується
глобальним поворотом. Тому цей графік краще показує navigation drift, а global
ATE лишається primary метрикою форми траєкторії.

## Відмінність від VIO та LIO

Поточний проєкт є loosely coupled odometry fusion. VO/Lidar спочатку самостійно
оцінюють relative transform, після чого EKF отримує тільки `dx,dy,dyaw`.

- **VIO** спільно оптимізує feature reprojection, IMU preintegration, poses,
  velocity і biases.
- **LIO** використовує IMU для deskew LiDAR scan та оптимізує point/plane
  residuals відносно local map разом з inertial state.

У реалізації немає IMU deskew, scan-to-map, loop closure або joint nonlinear
optimization. Тому 91 м на 10.7 км є помітним покращенням прототипу, але не
рівнем повноцінного LIO.

## Наступні покращення

1. IMU deskew для rolling VLP-16 scans.
2. Scan-to-map LiDAR odometry замість лише scan-to-scan ICP.
3. Online bias/noise adaptation окремо для straight та turning regimes.
4. VIO з IMU preintegration замість незалежного essential-matrix frontend.
5. Loop closure або map constraint для довгих міських петель.
6. Окрема оцінка на безперервних RTK segments, бо valid RTK покриває лише
   частину `urban39`.
