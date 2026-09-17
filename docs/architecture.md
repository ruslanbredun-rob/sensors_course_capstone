# Архітектура GPS-denied локалізації

## Статус і мета

Це **запланована** архітектура. Станом на створення документа є дані, design
document і каркас модулів; робочого фільтра, метрик і графіків ще немає.
ДЗ 18 завершується працюючим wheel + IMU EKF, який виводить стан на кожному
кроці. ДЗ 19 додає незалежну оцінку проти VRS-GPS, baseline, діагностику та
графіки. Slip-aware, stereo VO і LiDAR odometry — наступні окремі розширення.

## Потік даних

```text
urban35/sensor_data/* + calibration/*
              |
      dataset readers + validation
              |
     timestamp-ordered events (ns)
          /          |          \
       IMU        encoder     VO/LiDAR [пізніше]
        |             |             |
    predict      wheel update   pose update
          \          |          /
             2D EKF + NIS
                  |
        state(t), covariance(t), diagnostics
                  |
       result files + plots [ДЗ 19]
                  ^
                  |
VRS-GPS valid fix --> frame/time alignment --> RMSE
wheel-only baseline ---------------------------> comparison
```

VRS-GPS, FOG і `global_pose.csv` не входять до входів EKF. VRS є primary
reference лише при `fix_state = 4`; FOG можна використати для secondary yaw
check. `global_pose.csv` залежить від частини тих самих сенсорів, тому не є
незалежним ground truth.

## Межі модулів

| Модуль | Відповідальність | Вхід → вихід |
|---|---|---|
| `main.py` | CLI, вибір режиму та шляхів | аргументи → запуск pipeline |
| `src/config.py` | конфігурація і перевірка шляхів | JSON/CLI → `RunConfig` |
| `src/dataset.py` | читання CSV і calibration; перевірка колонок, одиниць і монотонності | файли → типізовані вимірювання |
| `src/synchronization.py` | впорядкування подій за nanosecond timestamp, tolerance, пропуски | потоки → події |
| `src/wheel_odometry.py` | counts → wheel speed та wheel-only baseline | encoder + calibration → швидкість/траєкторія |
| `src/ekf.py` | стан і коваріація; predict, updates, NIS | sensor events → estimates |
| `src/pipeline.py` | порядок викликів і режими E1–E4; не містить формул фільтра | config → results |
| `src/evaluation.py` | valid VRS, кадри, часові пари, RMSE і improvement | estimates + baseline + VRS → metrics |
| `src/visualization.py` | підписані trajectory/error/NIS графіки | results → PNG |

Stereo та LiDAR frontends згодом мають повертати однаковий контракт відносного
руху з timestamp і covariance, але їхні алгоритми залишаються окремими.
Конфігурація вибирає режим; модулі оцінювання не читають її напряму.

## Стан, кадри та час

- EKF володіє станом `[x, y, yaw, v, gyro_bias, accel_bias]` і `P`. Модулі
  readers не змінюють стан. Кути зберігаються в радіанах, відстані в метрах,
  швидкості в м/с, timestamps — цілі наносекунди.
- Локальний кадр автомобіля та світовий 2D кадр треба визначити з calibration
  до реалізації формул. VRS дає projected координати, але їхню проєкцію, вісь
  `x/y`, початок і lever arm треба перевірити до порівняння.
- Event loop обробляє вимірювання за часом. IMU запускає predict; encoder
  запускає update після перетворення counts. Кожний оброблений крок записує
  timestamp і стан. Для VO/LiDAR застосовується timestamp кадру; пропущені
  кадри не заповнюються за номером файлу.
- Некоректний `dt`, не монотонний час, відсутній файл чи невідома схема CSV
  мають давати явну помилку. Відсутній VO/LiDAR measurement пропускається,
  і EKF продовжує на IMU + wheel.

## Оцінювання та режими

**E1 Base:** wheel + IMU EKF і NIS. **E2 Slip-aware:** E1 із детекцією
пробуксовування та fallback. **E3 Visual:** E2 + stereo VO. **E4 Full:**
E3 + LiDAR odometry. Wheel-only dead reckoning — окремий raw baseline для
критерію ДЗ 19; E1–E4 порівнюються на однаковому наборі valid VRS епох.

Перед RMSE оцінка й VRS приводяться до одного кадру та зіставляються за часом
із фіксованою tolerance. Метрика охоплює всі valid reference епохи у часовому
перетині, а не лише останню точку. Виводяться RMSE baseline, RMSE EKF і
відносне покращення. Для консистентності записуються NIS та/або перевірка
додатної визначеності `P`; невалі́дні updates рахуються окремо.

## Точки розширення та ризики

- Вибір `Q/R`, модель IMU і slip thresholds належать EKF/measurement policy,
  а не CSV reader. Їхні значення мають бути в конфігурації після вимірювання
  характеристик даних.
- Важливий прихований зв'язок — calibration і координатні кадри. VO/LiDAR
  relative pose не можна прямо додати до світового стану без transform і
  коректної моделі вимірювання.
- NIS gating може приховати погану модель, якщо просто відкидати більшість
  updates; треба зберігати частку прийнятих вимірювань та інновації.
- Природне wheel slip у датасеті не розмічене. Для перевірки detector потрібна
  контрольована ін'єкція похибки й окрема оцінка false positives на вихідних
  даних.

## Артефакти фаз

| Фаза | Мінімальний результат |
|---|---|
| ДЗ 18 | `python main.py` читає IMU + encoder, синхронізує, реально змінює EKF і виводить стан; `requirements.txt` та README відповідають запуску |
| ДЗ 19 | `python main.py --mode all` відтворює baseline/EKF, RMSE на valid VRS, consistency diagnostics і PNG trajectory/error; слайди PDF готуються до захисту |

Архів здачі містить код, інструкції та результати. Датасет і його архіви не
включаються.
