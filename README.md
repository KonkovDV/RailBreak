<p align="center"><img src="Logo.png" alt="RailBreak" width="280"></p>

# RailBreak: резервная одометрия для автономного трамвая Москвы

Пакет для проверки — [`railbreak_backup_odometry`](railbreak_backup_odometry/).
ROS 2 Humble считает скорость и положение трамвая по двум тележкам и ручке
контроллера. GNSS читается только 3 с от первого валидного фикса любой антенны и нужен для выставки на карту,
дальше подписка снимается. Это резервный канал на случай, когда основной стек молчит, не замена лидара и не сертифицированный контур остановки.

Для жюри с этой страницы, из корня клона, при установленном Docker:

```text
scripts/jury.sh play --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
```

На Windows: `.\scripts\jury.ps1 play -Bag <каталог> -Msgs <tram_vehicle_msgs>`. Контейнер собирает пакет и проигрывает запись сам. Сеть хоста не нужна. Запись и пакет сообщений в репозиторий не входят. Ждать `/result/velocity` с первого сообщения тележки и `/result/position` после 3 с от первого фикса: `frame_id` `map`, `child_frame_id` `base_link`. Логи: `ros2 topic echo /result/diagnostics --field status[0].values`. Метрики записанного выхода: `python3 tools/organizer/score_ros.py <результат> <исходный bag> --frame mgrs`. `callback_max_us` — время одного колбэка, не задержка от входа до публикации.

Официальный checker из `check-code-with-bag.zip` собирается рядом с пакетом, в одном рабочем каталоге:

```text
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry hackathon_solution_checker
ros2 run railbreak_backup_odometry backup_odometry_node       # терминал 1
ros2 run hackathon_solution_checker metrics                   # терминал 2, итог после Ctrl+C
ros2 bag play <каталог rosbag2> --rate 1                      # терминал 3
```

`tram_vehicle_msgs` из этого архива не содержит `DriverControllerCommand.msg`. Без этого сообщения нода не стартует: ручка не подменяется нулём. Собирать нужно с полным пакетом сообщений организатора. Задержка вход→выход: `ros2 bag record` трёх входов и `/result/velocity` при `--rate 1`, затем `python3 tools/organizer/latency_probe.py --bag <запись>`. Скрипт берёт время приёма выхода минус время приёма входа с тем же `header.stamp`.

| Страница | Зачем |
|---|---|
| [Модель](docs/solution/model.md) | путь s, скорость v, таблица тяги, масштаб, якоря, целостность |
| [Допущения](docs/solution/assumptions.md) | единицы, параметры, GNSS-старт, карта, что не наблюдается |
| [Числа](docs/solution/results.md) | официальный checker, частота, разрывы, задержка; исторический ряд отдельно |
| [Разбор ТЗ](docs/solution/tz-audit.md) | что закрыто и какие ограничения остаются |

Запустить можно на Windows, Linux и macOS. Нода живёт в одном контейнере
Humble: Docker Desktop или Docker Engine, сеть хоста не нужна. Запись и пакет
сообщений организатора в репозиторий не входят, карта маршрута уже в пакете.
В `ci/tram_vehicle_msgs` лежит только интерфейс сборки: `VelocitySensor`
(`header`, `float64 velocity`) и `DriverControllerCommand` (`header`, `int8 position`).
Публичный GitHub Actions собирает ядро и ноду против этого интерфейса и гоняет unit-тесты. Запись он не проигрывает и пакет организатора не собирает. Проверка на записях — в [results.md](docs/solution/results.md).
Ряд 1.467 / 5.828 / 84.0 м — старое проверочное дерево `6af0037`. Фильтр после него менялся, и этот ряд не является результатом дерева, которое клонирует жюри. `current_replay_status` = `pending` значит только, что ряд не заменён.

Что показал прогон текущего дерева, 2026-09-27, коммит `5ed4a7c`, `--rate 1`, полный пакет сообщений организатора:

| Проверка | Итог |
|---|---|
| Официальный checker, `30618_88aea4d9`, положение 3D RMSE | 6.174 м (x 5.812, y 2.064, z 0.277) |
| Официальный checker, скорость RMSE | 0.067 м/с |
| Частота `/result/velocity` и `/result/position` | 29.3 Гц, разрыв не больше 0.051 с, штамп назад 0 раз |
| Задержка вход→выход | медиана 53 мс, p95 154 мс, максимум 306 мс |
| Память и CPU | 23.8 МБ RSS, 0.86 % одного ядра |
| `scripts/jury.sh acceptance`, `30618_e9a34502` | код 0 |

Подробности — [results.md](docs/solution/results.md).

## Проблема

Кейс — резервная скорость и путь для маршрута 10, когда GNSS потеряна или ей нельзя верить, а IMU, лидар и камеры во входах нет. Чтобы спланировать остановку, вагон должен знать, едет он или стоит и сколько пути уже прошёл. Штатные лидары, радары и камеры отказывают вместе: снег, грязь, помехи. Две тележки и ручка на вагоне уже есть.

Ориентир железной дороги, не наш допуск: ETCS SUBSET-041 пишет ±(5 м + 5 % пути) от репера. Этот прогон его не закрывает: официальный checker дал 6.174 м 3D RMSE на одной записи. ГОСТ Р 8.725 и уровень SIL здесь не заявляются. Граница вдоль пути в пакете — эмпирическая, не protection level.

## Идея

Состояние фильтра — [s, v, k, bₐ]: путь вдоль кольца, скорость, масштаб колёс относительно карты, смещение модели тяги. k — не уклон. Уклон берётся с карты отдельно.

Две тележки не независимы: одна шина, одно сцепление, одна карта. Отказ одной тележки раздувает её шум, отказ обеих одинаково фильтр не «усредняет», а помечает как ненаблюдаемый. GNSS ставит старт в MGRS и снимается. Стоянка у единственной остановки в воротах привязывает путь. Режимы целостности говорят, когда оценке уже нельзя верить: `NOMINAL`, деградация одной тележки или модели, `COMMON_MODE_UNOBSERVABLE`, затем `LOST`.

## Входы и выходы

| | Топик | Что приходит | Что делает нода |
|---|---|---|---|
| Вход | `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity` | `VelocitySensor`, в записях км/ч, около 10 Гц | перевод в м/с, `wheel_unit_scale = 1/3.6` |
| Вход | `/vehicle/driver_position_cmd` | `int8`, −15…+15 | строка таблицы тяги, не измерение ускорения |
| Вход | `/sensing/gnss/master/fix`, `.../rover/fix` | `NavSatFix` | только 3 с от первого валидного фикса, затем подписка уничтожается |
| Карта | `assets/` | кольцо, остановки, таблица ручки | не `tf` и не онлайн-SLAM |
| Выход | `/result/velocity` | м/с | с первого выпущенного сообщения тележки; на checker-записи около 29 Гц |
| Выход | `/result/position` | `Odometry`, x, y и z в MGRS, точка `base_link` | после окна GNSS; кадр `map` |
| Выход | `/result/diagnostics` | `DiagnosticArray` | режим, σₛ, `gnss=closed`, не JSON |

Без типа `DriverControllerCommand` нода не стартует и не подменяет ручку нулём.

## Алгоритм

Подробно — [model.md](docs/solution/model.md). Кратко, как считает нода, а не соседний UKF:

1. Прогноз. Ускорение — строка таблицы a(n, v) минус g·i(s) плюс смещение bₐ. Это не кривая момента и не акселерометр. На стоянке обеих тележек включается ZUPT.
2. Тележки. Плохую не выключаем: невязка выше порога получает R = 25 (м/с)². Если обе согласны и обе расходятся с моделью дольше 3 с, скорость колёс не берётся, режим `COMMON_MODE_UNOBSERVABLE`. Согласие двух тележек доверие не возвращает.
3. Якорь. Только стоянка и ровно одна остановка в воротах. Несколько кандидатов отклоняются. GNSS на станции путь не обнуляет: подписки уже нет. Чужая единственная станция в воротах может закрепиться.
4. Целостность. Пока слепой бюджет общей моды не выбран (5 с или 100 м от якоря), координата ещё выходит с низким доверием. После бюджета статус `LOST`, `/result/position` прекращается. Порог «σₛ > 5 м» таким правилом не является.
5. k на шаге колеса не двигается. Масштаб уточняют только якоря. В питче это заморозка k, не полный consider-фильтр.

## Почему так

Две тележки ловят отказ одной. Общий юз обеих они не ловят: это не два независимых канала и не SIL. Таблица тяги — медиана колёсного ускорения train по ручке и скорости, уклон возвращён. Паспорта двигателя в ней нет, и против общего юза на тесте она не третий датчик. Станции нужны потому, что по колёсам видно v/k, а не абсолютный путь. Одна координата вдоль известного кольца заменяет плоскую позу без IMU. Режимы вместо одного флага «исправен» для того, чтобы не ставить высокую уверенность на состояние, которое уже ненаблюдаемо.

## Результаты

Официальный checker, запись `30618_88aea4d9`, `--rate 1`, коммит `5ed4a7c`: положение 3D RMSE 6.174 м, скорость 0.067 м/с. Частота 29.3 Гц, разрыв штампов 0.051 с. Задержка вход→выход: медиана 53 мс, p95 154 мс, максимум 306 мс. Дольше 250 мс — 0.09 % выходов. RSS 23.8 МБ, CPU 0.86 % одного ядра. Медиана задержки в пределе 100 мс, хвост выше пика 250 мс.

1.467 м — медиана вдоль пути на старом дереве `6af0037`, не балл checker и не обещание, что текущий HEAD её улучшит. 9.48 м — ранний прогон `--rate 10` на прежней очереди, в этот итог не входит.

Свой `acceptance` на учебной `30618_e9a34502` закончился кодом 0. `score_ros.py` там даёт 3D 14.284 м против прокси антенн, не против `kinematic_state`.

## Ограничения

Одинаковая ошибка обеих тележек ненаблюдаема. Третий датчик (радар, IMU) в эту сдачу не входит. Карта — одно кольцо маршрута 10, без стрелок, депо и заднего хода. Другой вагон и другой сезон требуют заново снять таблицу и масштаб: вагон 30639 на той же карте ушёл примерно на 1.2 %. Между якорями ошибка пути на записи checker дошла до 20.7 м за ~2.7 км. Ложный якорь возможен, если в воротах одна чужая остановка.

## Опора

Постановка «путь вдоль известной геометрии, поза из карты» — Hasberg, Hensel, Stiller (IEEE T-ITS 2012) и von Einem и др. (ITSC 2023). Их оцениватели, IMU и несколько гипотез на стрелке в ноду не перенесены. SUBSET-041 и разборы Croydon и Salisbury задают, зачем резерв должен уметь сказать «не знаю». Уровень SIL по ним не назначается.

## Как запущено

ROS 2 Humble, C++17, Docker. Критический путь — эта нода в ROS, отдельного контура без ROS нет. Тесты ядра — CMake и `ctest`, не Google Test. Отдельной лицензии Apache 2.0 на код команды нет: передача организаторам по Положению, сторонний ROS 2 остаётся Apache 2.0. Потребление на прогоне checker — 23.8 МБ RSS и доля одного ядра, не отдельный стенд на 2 ГБ и 10 Вт.

Команды сборки и проигрывания — в начале этой страницы.

## После хакатона

Сначала то, что уже видно в ограничениях: сверка с выданным pathgraph, стрелки и депо, повтор таблицы на другом вагоне. Теневой прогон рядом с локализацией вагона имеет смысл только с записями и эталоном, которых в этом репозитории нет. Сертификация, SIL и тираж на парк в эту сдачу не входят.

Полуфинал хакатона — 30 сентября 2026, финал — 3 октября.

## Сценарии

Без Docker и без ROS собирается только ядро, сценарий `core`.

```text
scripts/jury.sh  <сценарий> --bag <каталог rosbag2> --msgs <tram_vehicle_msgs>
scripts/jury.ps1 <сценарий> -Bag <каталог rosbag2> -Msgs <tram_vehicle_msgs>
```

| Сценарий | Что делает |
|---|---|
| `smoke` | проигрывание обрывается через 25 с. Положение выходит, когда метка сообщения уходит дальше 3 с от первого валидного фикса любой антенны |
| `play` | вся запись, кадр MGRS, без `/clock` |
| `clock` | `use_sim_time` и `ros2 bag play --clock` |
| `fast` | `--rate 10` |
| `frame` | `output_frame:=mkrs_start` |
| `no-gnss` | в проигрыватель не попадают топики GNSS, выход относительный |
| `no-assets` | каталог карты подменён; `/result/position` есть, y = z = 0, x меняется, нода жива |
| `arc` | без GNSS, дуга старта `-InitialS` / `--initial-s` (по умолчанию 0) |
| `record` | пишет `/result/*` в `jury_out/result` и вызывает `score_ros.py`. Интерактивный прогон: код скорера становится кодом сценария, остальные проверки мягкие. `SCORE_MODE=exploratory` только предупреждает |
| `acceptance` | тот же прогон, но fail-closed: ненулевой код, если нода умерла, нет `/result/velocity`, нет `/result/position`, штамп регрессировал, частота уникальных штампов ниже 10 Гц, max gap выше 0.25 с, упал скорер, frame не `map`, child frame не `base_link`, `twist.linear.x` нет или NaN, GNSS не закрылся, или в дереве git появился новый `.db3` вне `jury_out` |
| `core` | CMake и тест ядра, без Docker, без записи и без сообщений |

Пример на Windows:

```text
.\scripts\jury.ps1 smoke -Bag D:\bags\30618_01f73500 -Msgs D:\dataset\tram_vehicle_msgs
```

Тот же вызов в bash. Первый запуск скачивает образ и собирает пакеты, это несколько минут.
Дальше сборка инкрементальная. `docker compose` из корня репозитория,
файл [`docker-compose.jury.yml`](docker-compose.jury.yml).

Ниже те же сценарии командами Humble на Ubuntu, если Docker нет.

| Что читать | Файл |
|---|---|
| Модель | [`docs/solution/model.md`](docs/solution/model.md) |
| Допущения и параметры | [`docs/solution/assumptions.md`](docs/solution/assumptions.md) |
| Точность и быстродействие | [`docs/solution/results.md`](docs/solution/results.md) |
| Разбор ТЗ | [`docs/solution/tz-audit.md`](docs/solution/tz-audit.md) |
| Текст полей формы | [`docs/solution/form.md`](docs/solution/form.md) |
| Питч | [`docs/solution/pitch.md`](docs/solution/pitch.md) |

## Контракт

Входы:

| Топик | Тип |
|---|---|
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/VelocitySensor` |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/DriverControllerCommand` |
| `/sensing/gnss/master/fix`, `/sensing/gnss/rover/fix` | `sensor_msgs/NavSatFix`, только окно старта |

Поле `velocity` тележки в записях — км/ч. Нода переводит его в м/с
(`wheel_unit_scale = 1/3.6`). Выход скорости — м/с. Отдельного топика тормоза
нет. Знак `position` ручки — это режим: больше нуля тяга, меньше нуля
торможение, ноль выбег. Нет отдельного топика тормоза — это не отказ. Нет типа
`DriverControllerCommand` — нода не стартует.

Три потока стоят в одной очереди. Выход несёт штамп входа, который очередь уже
выпустила: не новее минимума последних штампов живых потоков. Удержания сверх
этого по умолчанию нет (`stamp_reorder_s` = 0): внутри одного потока штамп назад не идёт.
Поток, который отстал, но ещё приходит, остаётся в минимуме. Замолчавший поток
выходит из минимума после 50 чужих входов; пока его отставание не больше 5 с,
знак не уходит дальше чем на 1 с от его последнего штампа.
Штамп, который после этого всё ещё позади последнего выхода, скорость и
положение не публикует. Он увеличивает `n_behind_out` и обновляет диагностику.

| Топик | Тип | Смысл |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | продольная скорость, м/с; с первого выпущенного сообщения тележки. На шаге нет, только если `velocity_confidence` равен `NONE` |
| `/result/position` | `nav_msgs/Odometry` | точка `base_link` в MGRS; после 3 с от первого валидного фикса любой антенны. На шаге нет, если `integrity_use_position` ложен |
| `/result/diagnostics` | `diagnostic_msgs/DiagnosticArray` | режим, `integrity_status`, `fault_score`, доверие скорости и положения, `k`, `s`, `σ_s`, GNSS, `callback_max_us`, `order_reason`, `n_behind_out` |

`base_link` — ось вращения первой тележки в точке касания колеса и рельса.
Кадр по умолчанию: UTM 37N минус угол квадрата 300000 м на восток и
6100000 м на север, x — восток, y — север. От антенны master точка сдвинута
на +9.873 м вдоль пути и на −3 м по высоте. tf: master `(−9.873, 0, 3)`,
rover `(2.563, 0, 3)`. Если в окне есть только rover, дуга старта отступает
на 12.436 м к master, и тот же сдвиг снова попадает в `base_link`.

`/result/velocity` — тот же тип, что у тележки, не `TwistStamped` и не
`Vector3Stamped`. Сообщение: `std_msgs/Header header` и `float64 velocity`.
`velocity` — продольная скорость фильтра, м/с; чекер из
`check-code-with-bag.zip` читает это поле и сравнивает его с
`twist.twist.linear.x` эталона. `header.frame_id` — `base_link`.
`header.stamp` — штамп входа, который породил публикацию (тележка или ручка),
не часы узла и не `/clock`.

Ориентиры ТЗ: задержка 100 мс (пик 250 мс), частота не ниже 10 Гц, не больше 2 ядер и 0.5 ГБ.
Замер текущего дерева — в таблице выше и в [`docs/solution/results.md`](docs/solution/results.md).
Отдельного таймера нет: без выпущенного входного сообщения выход не публикуется.
Диагностика пишется каждый `diagnostics_every_n` опубликованный выход (по умолчанию 20)
и на каждом штампе, который уже позади выхода. Числа 0.6–3.3 мс, 36–38 Гц и RSS 23–24 МБ
в той таблице сняты раньше текущего водяного знака.

Вход — best effort, очередь 500. Выход — reliable, очередь 10.
`ros2 topic echo` к выходу подключается. Надёжный подписчик к best-effort
издателю входа не подключается.

## Что должно быть установлено

Универсальный путь: Docker. Образ ставит Humble сам. На Windows и macOS это Docker Desktop,
на Linux — Docker Engine и плагин Compose. Каталог записи содержит `metadata.yaml`.

Путь без Docker — только Ubuntu 22.04 с уже установленным ROS 2 Humble
(`rclcpp`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `ament_index_cpp`, `launch_ros`, `ros2 bag`).
`rosdep` для пакета сдачи не нужен.

Сценарий `core` на любой ОС: CMake 3.16+, компилятор C++17. ROS не нужен.

## 1. Сборка на Ubuntu без Docker

Из корня клона. `<датасет>` — каталог выдачи, где лежит `tram_vehicle_msgs`.
Контейнерный путь этот шаг делает сам: `scripts/jury.sh` перед сборкой вставляет
`<maintainer>`, если в выданном `package.xml` его нет.

```bash
mkdir -p ~/ws/src
cp -a railbreak_backup_odometry ~/ws/src/
cp -a <датасет>/tram_vehicle_msgs ~/ws/src/
# В выданном package.xml нет <maintainer>, и catkin_pkg на Humble его отвергает.
source scripts/ensure_maintainer.sh
ensure_maintainer ~/ws/src/tram_vehicle_msgs/package.xml
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Проверка, что пакет виден:

```bash
ros2 pkg executables railbreak_backup_odometry
```

Ожидаются `backup_odometry_node` и `replay_events`.
Карта ставится в `install/railbreak_backup_odometry/share/railbreak_backup_odometry/assets`
(`ring.csv`, `stops.csv`, `notch.csv`).

Каждый новый терминал начинает с `source ~/ws/install/setup.bash`.

## 2. Обычный прогон записи

Ноду поднять до проигрывателя. Если запись уже идёт, окно GNSS в 3 с будет пропущено,
и положение останется относительным.

Терминал 1:

```bash
source ~/ws/install/setup.bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py
```

В логе строка вида `assets ...: ring ... m, ... stops`. Это карта загружена.

Терминал 2, после этой строки:

```bash
source ~/ws/install/setup.bash
ros2 bag play /путь/к/каталогу_bag --rate 1
```

`/clock` не нужен: интегратор читает `header.stamp` сообщения, не часы ROS.
Для сверки с лимитами ТЗ оставляйте `--rate 1`. Очередь выпускает вход только
до минимума последних штампов тележек и ручки.

Терминал 3:

```bash
source ~/ws/install/setup.bash
ros2 topic type /result/velocity
ros2 interface show tram_vehicle_msgs/msg/VelocitySensor
ros2 topic echo /result/velocity
ros2 topic hz /result/velocity
ros2 topic echo /result/position --field pose.pose.position
ros2 topic echo /result/diagnostics --field status[0].values
```

Что считать нормальным:

- `/result/velocity` появляется с первым выпущенным сообщением тележки;
- `/result/position` — после 3 с от первого валидного фикса любой антенны,
  x порядка 99–103 км, пока `integrity_use_position=true`;
- `gnss` становится `closed`: после окна старта GNSS больше не читается;
- `callback_max_us` — самое долгое время одного входа, микросекунды;
- `slip` — wheel/model inconsistency, model residual / slip proxy, не доказанное физическое проскальзывание. Расхождение могут дать slip, задержка, grade, ошибка traction table, изменение нагрузки, timestamp disorder и sensor fault. Статус `DEGRADED` включает сглаженный
  `fault_score`, не один NIS;
- режим `MODEL` — этот шаг недостоверен, положение ещё может выходить;
- режим `COMMON_MODE_UNOBSERVABLE` — обе тележки согласны и обе в юзе дольше
  3 с. Две согласные тележки не независимы, поэтому скорость с колёс в состояние не копируется и согласие друг с другом доверие не возвращает. Положение выходит, пока не
  выбран слепой бюджет: 5 с или 100 м от последнего якоря. Дальше
  `integrity_status=LOST`, `integrity_use_position=false`, `/result/position`
  на этих шагах нет. Скорость при этой потере остаётся: `velocity_confidence=LOW`. В диагностике `time_to_lost`, `distance_since_last_trusted_anchor` и `common_mode_exit`: `anchor` только после принятого якоря;
- `POSITION_UNTRUSTED` — штамп шагнул назад, обе тележки старше 30 с, или нет
  абсолютного старта. Положение на этом шаге не публикуется;
- `order_reason=ORDER_NOT_RESTORED` — один поток отстал больше чем на 1 с. Пока он приходит, выход его ждёт; после 50 чужих входов без него — нет.
  Образец в очереди не выбрасывается;
- `n_behind_out` — сколько штампов пришло уже позади опубликованного выхода.

Пропуск обеих тележек не останавливает модель: пока идёт ручка, выход ведёт таблица тяги.
Положение при этом публикуется, пока статус не `LOST` и не `POSITION_UNTRUSTED`.
Флаг проскальзывания лежит в диагностике, а не вместо координат.

## 3. Прогон с `/clock`

По умолчанию `use_sim_time` выключен, и так и надо, если проигрыватель без `--clock`.

Если нужен симуляционный час:

Терминал 1:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py use_sim_time:=true
```

Терминал 2:

```bash
ros2 bag play /путь/к/каталогу_bag --clock --rate 1
```

Штамп выхода по-прежнему равен штампу входа. Не включайте `use_sim_time` без `--clock`:
узлы, которые ждут `/clock`, не получат время.

## 4. Ускоренный прогон

```bash
ros2 bag play /путь/к/каталогу_bag --rate 10
```

Очередь входа — 500. На `--rate 10` стена в 10 раз короче штампов, поэтому
задержку для ТЗ снимают в сценарии 2.

## 5. Запись выхода и сравнение с антеннами

Терминал 1 — нода, как в сценарии 2. Терминал 2 — запись, её поднять до play:

```bash
source ~/ws/install/setup.bash
ros2 bag record -o /tmp/result_bag /result/velocity /result/position /result/diagnostics
```

Терминал 3:

```bash
ros2 bag play /путь/к/исходному_bag --rate 1
```

Остановить запись `Ctrl+C` после конца play. Сравнение из корня этого репозитория
(нужен Python 3 и `numpy`; читатель bag — стандартный `sqlite3`, ROS для скрипта не нужен):

```bash
python3 -c "import numpy"
python3 tools/organizer/score_ros.py /tmp/result_bag /путь/к/исходному_bag --frame mgrs
```

`--frame` должен совпадать с кадром ноды. Скрипт сравнивает выход с точкой
`base_link` на отрезке master→rover, не с антенной master. Сравнение с самой
антенной даёт около 10 м на ровном месте. Фикс rover входит в эталон, только
если планарная база master–rover лежит в пределах 12.436 ± 2 м и разность
высот антенн не больше 1 м: фикс в десятках метров от вагона — не антенна. В исходном bag нужны оба топика
`/sensing/gnss/master/fix` и `/sensing/gnss/rover/fix`; без rover скрипт завершается с кодом 2.

Таблица по всем выданным записям в [`docs/solution/results.md`](docs/solution/results.md)
посчитана офлайн тем же ядром. Команда `tools/organizer/eval_odometer.py` ждёт
локальное дерево записей и на чистом клоне не запускается.

## 6. Другой кадр выхода

Аргументы `ros2 launch`: `use_sim_time`, `assets_dir`, `output_frame`.
Остальные параметры — в `config/params.yaml` и в значениях по умолчанию ноды.
После правки yaml в исходниках нужна пересборка: launch читает файл из `install`.

Городская сетка от старта, первый отсчёт около нуля:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py output_frame:=mkrs_start
```

Допустимые значения `output_frame`: `mgrs` (по умолчанию), `mkrs_start`, `mkrs`,
`grid_start`, `enu`. Для `enu` и сеток от старта начало — точка карты в момент
выставки. Сдача — `mgrs`.

Свой каталог карты без пересборки:

```bash
ros2 launch railbreak_backup_odometry backup_odometry.launch.py assets_dir:=/путь/к/assets
```

В каталоге те же имена: `ring.csv`, `stops.csv`, `notch.csv`, `meta.yaml`.

Параметр, которого нет в launch, передаётся так:

```bash
ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
  -p output_frame:=mgrs \
  -p initial_s_m:=0.0
```

`ros2 run` берёт значения по умолчанию ноды. Они совпадают с `params.yaml`
для кадра, сдвигов и масштаба колеса. Пустой `assets_dir` означает карту из
share пакета.

## 7. Нет GNSS в начале записи

Если за первые 10 с нет ни одного фикса, выход положения — пройденный путь:
x растёт от первой входной метки, y = z = 0. Скорость публикуется как обычно.

Явная выставка без GNSS в записи:

```bash
ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
  -p initial_lat_deg:=55.810417 \
  -p initial_lon_deg:=37.462308
```

Либо дуга вдоль карты, в метрах: `-p initial_s_m:=1234.0`.
Пока эти параметры не заданы (NaN), нода их не подставляет сама.

Если master в окне пуст, а rover есть, старт берётся с rover и дуга отступает
на `rover_baseline_m` (12.436). Отдельного флага для этого нет.

## 8. Нет карты

Нода не падает. В логе `assets not loaded`. Скорость есть.
`/result/position.x` — пройденный путь, y = z = 0. GNSS-старт на пустую карту
не принимается.

```bash
ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
  -p assets_dir:=/nonexistent
```

## 9. Ядро без ROS

Так проверяется то же ядро, без Humble и без записи. Это сценарий `core`:

```text
scripts/jury.sh core
.\scripts\jury.ps1 core
```

Вручную из корня репозитория:

Linux:

```bash
cmake -S railbreak_backup_odometry/tools -B build/rbo -DCMAKE_BUILD_TYPE=Release
cmake --build build/rbo --parallel
ctest --test-dir build/rbo --output-on-failure
```

Windows (Visual Studio, конфигурация задаётся на сборке):

```bash
cmake -S railbreak_backup_odometry/tools -B build/rbo
cmake --build build/rbo --config Release
ctest --test-dir build/rbo -C Release --output-on-failure
```

Офлайн-повтор готового CSV тем же ядром, не замена сценария 2:

```text
replay_events <каталог assets> <events.csv> <out.csv>
```

Колонки `events.csv`: `t,kind,value,s0,sigma_s0`. `kind`: 0 — передняя тележка,
1 — задняя, 2 — ручка. Первая строка несёт начальную дугу `s0`.
Бинарь после сборки пакета: `ros2 run railbreak_backup_odometry replay_events`.

## Что не запускать как сдачу

`tram_dr_localization` и `docker-compose.yml` — прежний синтетический UKF,
версия `tramDR-0.0.11`. В фильтр сдачи он не входит, GNSS, IMU и lidar там
не используются, таблицы seed 42 не являются результатом маршрута 10.
Проверки того ядра — [`docs/verification.md`](docs/verification.md).

## Передача кода

Отдельной публичной лицензии нет: код передаётся организаторам.
Сторонние компоненты — [NOTICE](NOTICE).
