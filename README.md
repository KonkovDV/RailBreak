# RailBreak — резервная трамвайная одометрия

Оценка продольной скорости и относительного пути по ручке контроллера,
тормозной команде и скоростям колёс: физическая модель + ковариационный UKF
на C++17, ROS 2 Humble и независимые offline-инструменты проверки.
**GNSS, IMU и lidar не входят в online-фильтр.**

**Версия ветки: `tramDR-0.0.11`, пока не выпущена.** Изменения находятся в
[PR #5](https://github.com/KonkovDV/RailBreak/pull/5), не в `main`.
Это исследовательский прототип, не сертифицированная локализация и не
готовый компонент управления торможением. Реальные записи, целевая
платформа и HIL пока не подтверждают его эксплуатационную пригодность.

## Что изменено в этой порции

- Исправлен ремонт covariance у нижнего собственного значения:
  добавлен масштабируемый запас округления, сохранены проверка и atomic failure.
- Устойчиво вычисляется окно alpha для scaled UT, включая очень большие beta;
  отрицательные alpha отвергаются.
- Plant без указателя памяти больше не применяет и не теряет PT1/jerk каждый
  такт. Чистый выбег не разворачивается численно при пересечении нулевой скорости.
- Процесс массы выполняется на каждом принятом инициализированном шаге,
  включая silence и rest, а не только в обычном wheel-update.
- Добавлены три regression-набора; все **восемь CTest targets** автоматически
  включаются в ASan/UBSan. Версии C++ и ROS package синхронизированы.
- Исправлены HMI без экспозиции и coverage synthetic scorer до/после join;
  23 Python-теста проверяют знаменатели, пустые результаты и GT-сопоставление.
  Документы, ссылки и версии дополнительно проверяются offline-скриптом.
- Пересмотрены формулы и ограничения: наблюдаемость s, prior, NIS, ZUPT,
  зависимость канала A, диагностика PL/HMI и границы тестовых доказательств.

Конкретные baseline/fixed результаты и команды:
[docs/verification.md](docs/verification.md). Старые таблицы 0.0.10
не являются новыми измерениями 0.0.11.

## Быстрый старт без ROS

Нужны Git, CMake ≥ 3.16, компилятор C++17. Для offline-инструментов — Python
3.11+; основные тесты используют стандартную библиотеку.

```bash
git clone https://github.com/KonkovDV/RailBreak.git
cd RailBreak
# Пока PR #5 не слит: выбрать именно проверяемую ветку 0.0.11.
git switch audit/numerics-physics-2026-09-07

cmake -S standalone -B standalone/build -DCMAKE_BUILD_TYPE=Release
cmake --build standalone/build --parallel
ctest --test-dir standalone/build --output-on-failure

python3 tools/eval/test_eval.py
python3 tools/eval/test_metric_contracts.py
python3 tools/synth/test_generate.py
python3 tools/eval/no_gnss_scan.py
python3 tools/eval/test_docs.py
```

Synthetic e2e:

```bash
python3 tools/synth/generate.py --out synth/runs
python3 tools/eval/run_e2e.py --ukf standalone/build/replay_ukf
python3 tools/synth/score.py --runs synth/runs
```

Это synthetic twin, не полевой dataset. `mismatch_r0` — предусмотренный
отрицательный контроль и исключение из общего fail gate: зелёный e2e job
**не означает**, что все сценарии имеют нулевую ошибку или HMI.
Не сравнивайте результаты разных генераторов/профилей без фиксации provenance.

Для ASan/UBSan и точного состава suites:
[инструкция проверки](docs/verification.md).

## ROS 2 Humble

В подготовленной среде ROS 2 Humble с colcon и настроенным rosdep, из корня:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths tram_dr_localization --ignore-src -y --rosdistro humble
colcon build --packages-select tram_dr_localization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select tram_dr_localization
colcon test-result --all --verbose
```

Воспроизведение bag с `/clock`:

```bash
ros2 launch tram_dr_localization replay.launch.py bag:=/absolute/path/to/bag
```

Live без проигрывателя, с обычным временем:

```bash
ros2 launch tram_dr_localization live.launch.py
```

По умолчанию launch выбирает `vehicle_lvenok_moscow.yaml`. Для twin:

```bash
ros2 launch tram_dr_localization replay.launch.py \
  bag:=/absolute/path/to/bag \
  vehicle:="$PWD/tram_dr_localization/config/vehicle_combino_nf100.yaml"
```

Перед запуском проверьте DDS-типы, имена топиков, единицы, кодировку ручки,
радиус и количество каналов. `config/customer_topics.yaml` по умолчанию
выключает adapter, чтобы не зациклить уже нормализованный поток.
Смена имени топика не преобразует его тип. Подробности:
[архитектура](docs/architecture.md), [offline tools](tools/README.md).

Dockerfile/Compose присутствуют, но их runtime-проверка в этой порции
не выполнялась. Не путайте ROS build/gtests с проверкой работающего графа.

## Контракт оценки и ограничения

| Область | Что важно потребителю |
| --- | --- |
| Состояние | 12 координат, максимум 6 колёсных каналов; s — относительный путь, не абсолютная карта |
| Вход | SI; конечное `0 < dt ≤ 0.20 s`; валидный NaN/Inf команды отвергает шаг |
| Начало работы | До пригодных колёсных данных — UNINITIALIZED |
| Численный отказ | Откат шага, LOST и latch пути; восстановление скорости не восстанавливает потерянный путь |
| Odometry | `/tram/state_estimate` несёт s/v, но не готовую Cartesian pose; нужны diagnostics и исправление ROS pose-контракта |
| Целостность | Учитывать confidence, время и `s_unbounded`; нулевой `over_m` при latch не означает нулевую ошибку |
| ZUPT | Колёсные нули — эвристика, не независимое доказательство остановки кузова |
| Канал A | Нет прямой wheel-correction скорости, но параметры общие с UKF; канал не независим статистически |
| География | Пример route_10 — остановочная полилиния OSM, не обследованная ось пути; `/tram/fix` не GNSS fix |
| Скорость исполнения | Fixed-size core не доказывает hard realtime всей ROS-системы; WCET целевой платформы не измерен |

Требуют следующей инженерной порции: multirate sensor time/freeze,
валидация ROS-параметров и NaN brake, quaternion/frames/covariance,
GT-интерполяция и coverage bag-пути, strict-JSON export и runtime graph tests.
Список и критерии проверки: [verification.md](docs/verification.md).

## Как читать результаты

- Unit tests проверяют конкретные контракты; санитайзеры — некоторые классы
  ошибок памяти/UB, а не истинность физической модели.
- HMI-rate checker — условная доля среди matched OK; нужны также availability,
  coverage и число наблюдений. При нулевой экспозиции выводится N/A, не нулевой риск.
- Synthetic scorer сохраняет исходные количества GT/оценок, различает
  coverage GT-строк и долю matched estimates. Пустой файл оценки виден явно.
  Это не time-weighted coverage и не исправление отдельной интерполяции bag-пути.
- Nonnegative UT weights — достаточная численная политика, не необходимая
  теорема для любой функции и не доказательство калибровки covariance.
- Абсолютный сдвиг s не наблюдаем по колёсам, но cross-covariance позволяет
  уточнять относительный путь и уменьшать P_ss.
- Stationary variance prior относится к изолированному процессу log(m),
  не ко всей posterior UKF. NIS считается до Huber, но после адаптаций R.
- PL/AL и synthetic HMI не устанавливают SIL, THR или безопасность маршрута.

## Документация

| Документ | Назначение |
| --- | --- |
| [Кратко о решении](docs/brief.md) | Возможности, принципы и непокрытые режимы |
| [Математика](docs/math.md) | Силы, интегратор, UT, covariance, наблюдаемость |
| [Априор и NIS](docs/estimator-priors.md) | Формулы, расписание процесса массы, пределы статистик |
| [Архитектура](docs/architecture.md) | Компоненты, время, конфигурация, ROS-вход/выход |
| [Проверки](docs/verification.md) | CTest, Python-контракты, CI, TDD и открытая матрица |
| [Checker](docs/checker.md) | Реальные классы нарушений, GT join и знаменатели метрик |
| [Целостность и риск](docs/integrity-risk.md) | Диагностики, latch и отсутствие safety-гарантий |
| [Метрики 0.0.10](docs/metrics.md) | Исторические synthetic результаты, не измерения 0.0.11 |
| [Исторический аудит](docs/audit-2026-09-06.md) | Происхождение старых findings |
| [Источники](docs/refs.md) | Библиография; не замена валидации реализации |

Исторические review/pitch/evidence и старый changelog следует читать в
контексте их версии. При расхождении описания текущего поведения используйте
актуальные контракты выше и исходники нужного commit, а не старые обещания.

## Лицензии и данные

Код распространяется на условиях [LICENSE](LICENSE); сторонние уведомления
и условия распространения материалов перечислены в [NOTICE](NOTICE).
Пример геоданных OSM имеет отдельные условия ODbL. Лицензия кода не
предоставляет автоматически права на чужие записи, карты или материалы
заказчика. Проверяйте права и конфиденциальность до публикации/сдачи.
Вложения из частной переписки в рамках этого аудита в репозиторий не добавлялись.
