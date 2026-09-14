# ДЗ17 — проєктний документ сенсорної системи

**Назва проєкту:** GPS-denied локалізація автомобіля на Complex Urban Dataset  
**Студент:** Руслан Бредун  
**Дата:** 2026-09-14

## 1. Use case та постановка задачі

**Use case:** локалізація наземного автомобіля в щільній міській забудові, де GNSS
недоступний або ненадійний через екранування та multipath.

Система оцінює плоский стан автомобіля `(x, y, yaw, v)` за колісними енкодерами та
IMU без подавання GPS у фільтр. Базовий 2D EKF включає NIS-контроль інновацій. Далі
послідовно додаються детекція пробуксовування коліс, stereo visual odometry та
LiDAR odometry. Кожний етап оцінюється окремо, щоб кількісно показати внесок нового
механізму або сенсора.

VRS-GPS (RTK) використовується лише як незалежний reference на епохах із якісним
fix. FOG використовується як додатковий reference кута курсу. `global_pose.csv` можна
показувати лише як вторинну SLAM-траєкторію, оскільки вона побудована із частини
тих самих сенсорів і не є незалежним ground truth.

## 2. Вимоги до системи

| Параметр | Цільове значення | Обґрунтування |
|---|---:|---|
| Вихідний стан | `x, y` [м], `yaw` [рад], `v` [м/с] | Мінімальний стан для 2D локалізації автомобіля |
| Частота стану EKF | не менше 100 Hz | Відповідає фактичній частоті IMU/encoder у `urban35` |
| Частота VO та LiDAR corrections | 10 Hz | Частота stereo і VLP-16 у датасеті |
| Primary position metric | 2D RMSE/ATE на всіх епохах `VRS fix_state = 4` | Метрика рахується лише там, де є надійний незалежний reference |
| Ціль final pipeline | RMSE не більше 10 м і щонайменше на 20% нижче Base | Перевіряє абсолютну якість і реальну користь додаткових сенсорів |
| Consistency | NIS test, `p = 0.95`; близько 95% raw innovations нижче відповідного χ² threshold | Виявляє неузгоджені `Q/R`, outliers і несправні вимірювання |
| Відмова wheel update | без падіння програми; `R_wheel` збільшується або update відкидається | Явний degraded mode при пробуксовуванні |
| Slip detection | detection rate ≥90%, false-positive rate ≤5% на інжектованих fault intervals | У датасеті немає готової розмітки природного wheel slip |
| Відтворюваність | одна команда `python main.py --mode all` | Вимога курсу до перевірки на чистому середовищі |
| Обчислювальний бюджет | RAM до 16 GB; повний offline run до 30 хв на звичайному ноутбуці | Дає практичне обмеження для VO, ICP та експериментів |
| Сховище | до 6 GB для `urban35`; dataset не входить у ZIP/Git | Поточна локальна копія займає близько 5.5 GB |
| Середовище | щільна міська забудова, повороти, динамічні об'єкти, зміни освітлення | Основні умови та джерела деградації Complex Urban Dataset |

## 3. Sensor Allocation Table

| # | Сенсор | Модель | Роль у проєкті | Формат / частота | Вартість для проєкту |
|---:|---|---|---|---|---:|
| 1 | Два rear-wheel енкодери | RLS LM13, 4096 counts/rev | Швидкість, пройдений шлях і differential yaw; Base | CSV, 100 Hz | $0, open dataset |
| 2 | IMU/AHRS | Xsens MTi-300 | Angular rate, acceleration, EKF prediction; Base | CSV, близько 100 Hz у `urban35` | $0 |
| 3 | Stereo camera | 2× FLIR FL3-U3-20E4C-C | Metric stereo VO; Visual stage | PNG 1280×560, 10 Hz | $0 |
| 4 | 3D LiDAR | 2× Velodyne VLP-16 | Scan matching / ICP relative motion; Full stage | BIN, 10 Hz | $0 |
| 5 | VRS-GPS | SOKKIA GRX2 | Незалежний position reference; не входить у fusion | CSV, 1 Hz | $0 |
| 6 | 3-axis FOG | KVH DSP-1760 | Secondary yaw reference; не входить у main fusion | CSV, 1000 Hz | $0 |

Два енкодери дають точну короткочасну оцінку руху вздовж дороги, але не бачать
пробуксовування. IMU вимірює швидкі зміни прискорення та кутової швидкості, але її
інтегрування накопичує bias і drift. Stereo VO додає незалежне від коліс metric
relative motion, а LiDAR scan matching дає геометричне обмеження за слабкої
текстури або зміни освітлення. VRS і FOG навмисно утримуються від основного fusion,
щоб evaluation не використовував ті самі вимірювання, що й оцінювач.

## 4. Block diagram

```text
 Complex Urban Dataset / urban35
              |
      loaders + timestamp sync
              |
   +----------+------------------+------------------+
   |                             |                  |
 encoder.csv                xsens_imu.csv      stereo PNG / VLP BIN
   |                             |                  |
 wheel odometry ----------> 2D EKF <--------- stereo VO / LiDAR ICP
   |                        predict/update          |
   +--> slip detector            |             NIS gating
        |                         |
        +--> wheel R increase     +--> estimated x, y, yaw, v
             or reject update               |
                                           evaluation
                           +-----------------+----------------+
                           |                 |                |
                     VRS fix=4          FOG yaw       global_pose.csv
                    primary RMSE       secondary       secondary only
```

NIS входить уже до **Base**: для кожного wheel measurement EKF обчислює інновацію
`ν`, її covariance `S` та `NIS = νᵀS⁻¹ν`. Для scalar wheel-speed update поріг
`χ²(0.95, 1) = 3.841`. Для 3D relative-pose update `(Δx, Δy, Δyaw)` поріг
`χ²(0.95, 3) = 7.815`. Перевищення порога реєструється, а вимірювання відкидається
або його `R` тимчасово збільшується.

## 5. Resource та power budget

Проєкт обробляє вже записаний датасет і не проєктує бортове живлення стенда.
Тому струм сенсорів, duty cycle та час роботи батареї не впливають на реалізацію,
а їх підстановка без схеми живлення автомобіля була б вигаданою. Практичний бюджет
проєкту задається ресурсами offline pipeline:

| Компонент | Duty cycle | Практичне обмеження |
|---|---:|---:|
| Wheel + IMU EKF | 100% sequence | real-time factor ≤ 0.1 |
| Stereo VO | 10 Hz frames | до 8 GB RAM |
| LiDAR preprocessing + ICP | 10 Hz scans | до 12 GB RAM |
| Повний pipeline | один `urban35` run | до 16 GB RAM, до 30 хв |
| Sensor procurement | — | $0: використовується відкритий датасет |

Якби pipeline переносився на реальний автомобіль, power budget і вартість сенсорів
потрібно було б розрахувати окремо для обраного compute unit, камер, двох LiDAR та
DC/DC перетворювачів.

## 6. FMEA

| Джерело | Режим відмови | Ефект | Severity, 1–5 | Detection / mitigation |
|---|---|---|---:|---|
| Encoder | Пробуксовування або завислий count | Хибна швидкість, position/yaw drift | 4 | Slip flag за розбіжністю wheel–IMU; NIS; збільшити `R_wheel` або пропустити update |
| IMU | Bias, spike, saturation | Помилка yaw і швидке накопичення drift | 5 | Bias states, range checks, NIS на пов'язаних updates, обмеження `dt` |
| Timestamp | Пропуск або несинхронний sample | Fusion фізично різночасних вимірювань | 4 | Сортування, monotonicity check, nearest timestamp із tolerance |
| Stereo | Мало features, blur, зміна освітлення | Ненадійний або відсутній VO increment | 3 | RANSAC/inlier threshold, NIS gating, fallback до slip-aware Base |
| LiDAR | Degenerate ICP, динамічні об'єкти | Помилковий relative pose | 4 | Fitness/RMSE gate, motion bound, NIS, fallback без LiDAR update |
| VRS-GPS | No fix або multipath | Відсутня/хибна reference epoch | 3 | Метрика лише при `fix_state = 4`; `no fix = no metric` |
| Dataset | Відсутній перший right frame | Зсув stereo/LiDAR пар на один індекс | 4 | Pairing лише за timestamp/filename, ніколи за номером масиву |

## 7. Data source

Використовується **Complex Urban Dataset, sequence `urban35` (Seoul, близько
3.2 км)**. Датасет обрано через одночасну наявність реальних wheel encoders, IMU,
FOG, stereo camera, 3D LiDAR і VRS-GPS у складному міському середовищі. Такий набір
дозволяє дослідити саме GPS-denied dead reckoning і поступове додавання незалежних
relative-motion constraints. Використання цього джерела та правило evaluation
тільки на якісних RTK-епохах погоджені з лектором.

Локальні дані розташовані в `data/complex_urban/urban35` і не додаються до Git або
архіву здачі. У `data/README.md` наведено посилання, очікувану структуру і відомі
пропущені кадри. Усі потоки синхронізуються за nanosecond timestamps, а не за
індексами рядків або файлів.

## 8. Planned pipeline та експерименти

Спочатку encoder counts перетворюються на швидкості коліс з використанням відомих
діаметрів і бази автомобіля. IMU виконує predict для 2D EKF зі станом
`[x, y, yaw, v, gyro_bias, accel_bias]`, а wheel velocity виконує update з NIS
test. Наступний етап додає slip detector і fallback для wheel update. Після цього
stereo frontend оцінює metric relative pose, а LiDAR frontend — relative pose через
preprocessing та ICP; кожний update проходить quality checks і NIS gating.

| Експеримент | Конфігурація | Що перевіряється |
|---|---|---|
| E1 Base | Wheel + Xsens IMU + 2D EKF + NIS | Dead-reckoning baseline і consistency |
| E2 Slip-aware | E1 + slip detection/fallback + контрольований encoder fault injection | Detection rate, false alarms і стійкість до ненадійної wheel odometry |
| E3 Visual | E2 + stereo VO | Внесок незалежного camera motion constraint |
| E4 Full | E3 + LiDAR odometry | Внесок geometric scan matching |

Для E2 у відомі часові інтервали реальних encoder counts програмно додається
масштабна помилка, що імітує пробуксовування. Незмінені дані також запускаються для
оцінки false positives. Для кожного експерименту зберігаються trajectory, 2D
RMSE/ATE на valid VRS epochs, final drift, yaw error проти FOG, NIS statistics та
runtime. Основний результат — таблиця E1–E4 і графіки trajectory, position error
та NIS. Tightly coupled VIO і LIO залишаються advanced extensions поза основним
scope та виконуються тільки якщо готові основний pipeline, evaluation і матеріали
захисту.

## Джерела

- [Complex Urban Dataset — офіційна сторінка](https://sites.google.com/view/complex-urban-dataset)
- [Опис датасету та сенсорної конфігурації](https://journals.sagepub.com/doi/10.1177/0278364919843996)
- [Xsens MTi-300 datasheet](https://www.xsens.com/hubfs/Downloads/Leaflets/MTi-300.pdf)
