# Результати та висновки ДЗ 18–19

## Умови оцінки

VRS-GPS не надходить у EKF. Для оцінки беруться тільки `fix_state=4`; reference
epochs зіставляються з найближчим state у межах 50 мс.

Primary metric — global rigid SE(2) ATE без scale fit. Initial-pose metric
суміщає старт і визначає yaw за першим надійним відрізком руху близько 20 м.
Вона вища, бо не компенсує accumulated yaw drift поворотом усієї траєкторії.

| Характеристика | `urban35` | `urban33` | `urban39` |
|---|---:|---:|---:|
| Тривалість | 173.9 с | 1284.4 с | 1866.8 с |
| Encoder samples | 17 388 | 128 436 | 186 675 |
| IMU samples | 17 388 | 128 442 | 186 682 |
| Валідні RTK epochs | 169 | 908 | 314 |
| Безперервні RTK segments | 2 | 12 | 10 |
| Сумарна тривалість RTK segments | 167.0 с | 890.0 с | 298.0 с |
| Wheel distance | ≈3.2 км | ≈7.4 км | ≈10.7 км |

Послідовності наведені від короткої до довгої. `urban35` майже прямий;
`urban33` і `urban39` мають більше поворотів. На `urban39` valid RTK покриває
лише близько 298 із 1867 секунд, тому whole-trajectory metric фактично описує
десять окремих інтервалів.

## `urban35`: короткий майже прямий маршрут

| Конфігурація | Global RMSE, м | Median, м | P95, м | Initial-pose RMSE, м |
|---|---:|---:|---:|---:|
| Adaptive IMU + wheel | 3.978 | 3.205 | **6.225** | **50.723** |
| Baseline + stereo VO | 3.978 | 3.205 | **6.225** | **50.723** |
| Baseline + LiDAR | **3.259** | **2.502** | 6.355 | 57.570 |
| Full | **3.259** | **2.502** | 6.355 | 57.570 |

![Траєкторії urban35](urban35/screenshots/trajectory_comparison.png)

LiDAR зменшує global RMSE на 18.1%. Stereo VO має coverage нижче 30% і тому не
входить у fusion; visual mode дорівнює baseline. Низький global RMSE означає,
що форма майже прямого маршруту добра. Значно більший initial-pose RMSE виникає
через малу похибку початкового кута: на дистанції 3.2 км навіть близько одного
градуса дає десятки метрів lateral error.

## `urban33`: довший маршрут з поворотами

| Конфігурація | Global RMSE, м | Median, м | P95, м | Initial-pose RMSE, м |
|---|---:|---:|---:|---:|
| Adaptive IMU + wheel | 68.508 | **32.095** | **118.764** | **105.781** |
| Baseline + stereo VO | 72.190 | 32.860 | 136.079 | 147.014 |
| Baseline + LiDAR | **67.932** | 36.390 | 130.649 | 297.383 |
| Full | 68.298 | 36.440 | 131.554 | 303.562 |

![Траєкторії urban33](urban33/screenshots/trajectory_comparison.png)

LiDAR покращує global RMSE лише на 0.8%; stereo VO погіршує його на 5.4%.
Global fit компенсує середній yaw offset, тому LiDAR може мати схожий global
RMSE і водночас гіршу initial-pose metric. Це означає, що frontend змінює
довготривалу орієнтацію, але не робить її стабільно точнішою на всіх поворотах.

## `urban39`: найдовший маршрут і неповний RTK reference

| Конфігурація | Global RMSE, м | Median, м | P95, м | Initial-pose RMSE, м |
|---|---:|---:|---:|---:|
| Adaptive IMU + wheel | 180.640 | 102.853 | 335.998 | **387.142** |
| Baseline + stereo VO | 190.904 | 105.672 | 356.284 | 401.797 |
| Baseline + LiDAR | **176.724** | **97.722** | **332.078** | 388.932 |
| Full | 176.730 | 97.726 | 332.088 | 388.933 |

![Траєкторії urban39](urban39/screenshots/trajectory_comparison.png)

LiDAR дає 2.2% покращення global RMSE, stereo VO погіршує його на 5.7%.
Різниця між LiDAR і full практично відсутня, бо LiDAR покриває майже всі frame
pairs, а VO в `full` використовується тільки у його прогалинах.

200 м похибки на маршруті близько 10.7 км пов'язані насамперед із yaw drift.
Малий систематичний yaw-rate offset накопичується на численних поворотах і
перетворюється на велику position error. Неповний RTK coverage додатково робить
одне whole-run число менш репрезентативним; деталізація є в
[`rtk_segment_metrics.csv`](urban39/rtk_segment_metrics.csv).

## Як обробляються дані

1. Readers читають headerless CSV, перевіряють fields, timestamps, SI ranges та
   calibration files.
2. Encoder cumulative counts диференціюються за фактичним `dt`. Wheel diameter,
   resolution та wheelbase дають signed left/right speed, forward speed і
   differential yaw rate.
3. IMU gyro z та acceleration x виконують EKF prediction. Wheel measurements
   коригують speed і gyro bias.
4. За yaw rate рух класифікується як `straight` або `turning`. Для кожного
   режиму окремо адаптуються speed/yaw measurement noise та gyro-bias random
   walk за EMA від NIS.
5. Stereo VO повертає metric body-frame `dx,dy,dyaw` після rectification,
   ORB/RANSAC, disparity scale та camera-to-vehicle transform.
6. LiDAR points переводяться у vehicle frame. Wheel speed та IMU yaw rate
   виконують deskew, після чого scan-to-scan і scan-to-local-map ICP оцінюють
   relative pose.
7. EKF зберігає pose clone, covariance та cross-covariance на попередньому
   frontend epoch. Relative innovation коригує `x,y,yaw`.
8. Relative-pose NIS вище soft threshold збільшує measurement covariance.
   Measurement відкидається тільки коли потрібний scale перевищує 100.
9. Після estimator run VRS використовується для global, initial-pose та
   continuous-segment evaluation. Він не впливає на state.

## Вплив сенсорів

- **IMU + wheels** є обов'язковим baseline. IMU добре відтворює швидкі зміни yaw,
  але не дає стабільної лінійної швидкості після подвійної інтеграції. Колеса
  дають speed і кінематичний course, але мають систематичну похибку на поворотах.
- **Stereo VO** у поточному essential-matrix frontend має scale scatter і
  sequence bias. NIS прибирає окремі outliers, але не постійний bias, тому VO не
  покращує ці три sequences.
- **LiDAR** має високе coverage і стабільніший relative transform. IMU deskew
  зменшує motion distortion, а local map додає геометрію кількох scans. На цих
  даних покращення лишається малим: коротка локальна карта не усуває глобальний
  yaw bias і не має loop closure.
- **Full** майже повторює LiDAR-only, оскільки VO та LO корельовані, а LiDAR має
  пріоритет на спільних intervals.

Окремий binary slip detector прибрано. На реальних sequences він майже не
активувався й іноді погіршував результат пропуском корисних wheel updates.
Wheel inconsistency тепер враховується без окремого state через NIS gating і
regime-specific adaptive covariance.

## Чому scan-to-map не дав великого покращення

Чистий scan-to-map варіант надто залежав від wheel/IMU initial guess: local map
рухалася разом із накопиченим yaw drift. На прямій це майже непомітно, а на
довгій послідовності з поворотами карта зберігала систематичну помилку.

Поточний frontend тому гібридний. Scan-to-scan ICP дає основну локальну delta,
scan-to-map ICP стабілізує її картою останніх трьох scans, а їхній внесок явно
задається covariance та `map_measurement_weight`. Це не повноцінний LIO:
відсутні point-to-plane residuals, joint IMU optimization, loop closure і
глобальний map constraint.

## RTK segments

Gap понад 1.5 с починає новий segment; segments коротші за п'ять epochs не
входять у таблицю. Кожен рядок містить start/end timestamp, duration, RMSE та
final error. Errors беруться з єдиного whole-trajectory alignment і не
перевирівнюються окремо для кожного segment, тому між сегментами зберігається
накопичений drift.

Це особливо важливо для `urban39`: 314 valid fixes утворюють десять segments із
сумарною тривалістю близько 298 с. Оцінка не доводить точність на частинах
маршруту, де valid RTK відсутній.

## Подальші покращення

Реалізовано IMU deskew, коротку scan-to-map local map, online regime adaptation
та окремі RTK segment metrics. Найкорисніші наступні кроки за зростанням обсягу:

1. Використати точні VLP-16 firing timestamps замість оцінки часу за порядком
   points.
2. Додати point-to-plane residuals і довшу local submap з контрольованою
   marginalization.
3. Додати online calibration effective wheelbase та gyro scale на поворотах.
4. Реалізувати VIO з IMU preintegration замість незалежного essential-matrix
   frontend.
5. Додати loop closure або зовнішній map constraint для довгих міських петель.
