# Результати та висновки ДЗ 18–19

## Умови порівняння

VRS-GPS не надходить у EKF. Для оцінки беруться тільки `fix_state=4`, VRS
epochs зіставляються зі state за timestamp у межах 50 мс.

Primary metric — global rigid SE(2) ATE без scale fit. Окремо рахується start
anchored error: yaw береться з того самого SE(2) fit, але translation суміщає
перші точки. На trajectory plots використано start anchored coordinates, тому
початок estimate та reference збігається.

| Характеристика | `urban35` | `urban39` |
|---|---:|---:|
| Тривалість | 173.9 с | 1866.7 с |
| Encoder samples | 17 388 | 186 675 |
| IMU samples | 17 388 | 186 682 |
| Валідні RTK epochs | 169 | 314 |
| Polyline по valid RTK epochs | 3215 м | 5577 м |
| Повна довжина sequence | ≈3.2 км | ≈11.1 км |

`urban35` — короткий майже прямий маршрут. `urban39` триває понад 31 хвилину
і містить багато поворотів та петель.

## Результати `urban35`

| Конфігурація | Global RMSE, м | Median, м | P95, м | Start RMSE, м |
|---|---:|---:|---:|---:|
| E1 IMU + wheel | 4.153 | 3.104 | 7.024 | 7.731 |
| E2 kinematics + slip | **3.788** | 3.259 | 5.958 | **7.119** |
| E2 + stereo VO | 3.788 | 3.259 | 5.958 | 7.119 |
| E2 + LiDAR | 3.788 | 3.259 | 5.958 | 7.119 |
| E2 + stereo VO + LiDAR | 3.788 | 3.259 | 5.958 | 7.119 |

![Траєкторії urban35](urban35/screenshots/trajectory_comparison.png)

E2 знижує global RMSE на 8.8%. Stereo frontend прийняв 184/868 пар, LiDAR —
27/1723. Coverage становить 21.2% та 1.6%, нижче health threshold 30%, тому
розріджені relative transforms не подаються у EKF. Через це VO/LO modes
зберігають результат E2. На майже прямому маршруті це очікувано: Wheel+IMU вже
добре відтворює форму траєкторії, а frontends не дають безперервного constraint.

## Результати `urban39`

| Конфігурація | Global RMSE, м | Median, м | P95, м | Start RMSE, м |
|---|---:|---:|---:|---:|
| E1 IMU + wheel | 188.456 | 108.539 | 348.793 | 397.011 |
| E2 kinematics + slip | 188.471 | 108.484 | 348.879 | 397.094 |
| E2 + stereo VO | 197.477 | 110.616 | 367.559 | 417.832 |
| E2 + LiDAR | 90.789 | 57.316 | 168.823 | **168.629** |
| E2 + stereo VO + LiDAR | **90.690** | **57.029** | **168.689** | 168.668 |

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
9. Після estimator run VRS використовується для global та start anchored
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

## Чому перший графік не збігався на старті

Попередній plot показував global Kabsch alignment. Він мінімізує сумарну ATE і
може змістити першу точку; на старому Urban39 графіку цей offset становив
приблизно 349 м. Це була властивість візуалізації, а не timestamp shift.

Тепер trajectory plots використовують start anchored translation, тому старт
збігається. Primary global ATE збережено окремо, щоб не змінювати стандартну
метрику.

## Відмінність від VIO та LIO

Поточний проєкт є loosely coupled odometry fusion. VO/Lidar спочатку самостійно
оцінюють relative transform, після чого EKF отримує тільки `dx,dy,dyaw`.

- **VIO** спільно оптимізує feature reprojection, IMU preintegration, poses,
  velocity і biases.
- **LIO** використовує IMU для deskew LiDAR scan та оптимізує point/plane
  residuals відносно local map разом з inertial state.

У реалізації немає IMU deskew, scan-to-map, loop closure або joint nonlinear
optimization. Тому 91 м на 11.1 км є помітним покращенням прототипу, але не
рівнем повноцінного LIO.

## Наступні покращення

1. IMU deskew для rolling VLP-16 scans.
2. Scan-to-map LiDAR odometry замість лише scan-to-scan ICP.
3. Online bias/noise adaptation окремо для straight та turning regimes.
4. VIO з IMU preintegration замість незалежного essential-matrix frontend.
5. Loop closure або map constraint для довгих міських петель.
6. Окрема оцінка на безперервних RTK segments, бо valid RTK покриває лише
   частину `urban39`.
