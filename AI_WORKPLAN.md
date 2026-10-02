# RailBreak — план работы для ИИ

Этот план предназначен для следующего ИИ/инженера. Он разделяет проверяемые факты, обязательные эксперименты и недоказанные гипотезы.

## 0. Правила работы

- Не считать старые screenshots, `result.json` и historical metrics доказательством текущего кода.
- Перед каждым benchmark фиксировать commit/tree, SHA-256 assets, params, rosbag, command line, ROS/Docker image и timezone.
- GNSS использовать как начальную/корректирующую/reference информацию только там, где это явно разрешено сценарием; не подменять им нормальную одометрию.
- Любое изменение ROS topic/type/frame/timestamp считать API-изменением и покрывать контрактным тестом.
- Не называть interval safety bound, percentile, certification или production readiness без отдельного доказательства.
- При ошибке assets, message type, QoS или времени предпочитать явный отказ и диагностику тихой деградации.

## 1. Воспроизвести чистую сборку

1. Создать чистый ROS 2 Humble workspace без файлов из старого `build/`, `install/`, `log/`.
2. Скопировать пакет и `ci/tram_vehicle_msgs` в workspace.
3. Проверить `package.xml`, `CMakeLists.txt`, install rules и `ament_package`.
4. Выполнить offline:
   - `rosdep install --from-paths src --ignore-src --rosdistro humble --default-yes` в заранее подготовленном environment;
   - `colcon build --symlink-install --event-handlers console_direct+`;
   - `colcon test --event-handlers console_direct+`;
   - `colcon test-result --verbose`.
5. Повторить сборку с доступным полным `tram_vehicle_msgs` и с check-code variant, где `DriverControllerCommand` отсутствует.
6. Убедиться, что generated-header detection не использует stale header из `install/`.

**Артефакты:** `build.log`, `test-result.log`, `colcon-list.txt`, compiler/version output и полный environment record.

## 2. Проверить ROS интерфейс

Проверить фактические типы, поля и units для:

- `/vehicle/front_bogie_velocity`;
- `/vehicle/rear_bogie_velocity`;
- `/vehicle/driver_position_cmd`;
- optional `/sensing/gnss/master/fix`, `/sensing/gnss/master/vel`, `/sensing/gnss/rover/fix`, `/sensing/gnss/rover/vel`;
- `/result/velocity`;
- `/result/position`;
- `/result/diagnostics`.

Для каждой пары topic/type записать:

- message type и используемое поле;
- единицы и преобразование;
- header stamp/source stamp;
- frame id и child frame;
- QoS history/depth/reliability/durability;
- поведение при NaN, stale и equal timestamp.

Сделать отдельные тесты на:

- отсутствие `DriverControllerCommand`;
- наличие `DriverControllerCommand`;
- неизвестный/неполный message type;
- duplicate/equal/regressed timestamps;
- отсутствие GNSS после стартового окна;
- output rate не ниже требуемой при редких input callbacks.

## 3. Проверить время и порядок

1. Запустить `InputReorder` на синтетических потоках с разными rate, burst, stall и backlog.
2. Проверить watermark, `stamp_reorder_s`, `order_stall_s`, silent-push release и отсутствие потери sample внутри очереди.
3. Снять input-to-output latency для каждого callback type и extrapolation path.
4. Проверить отдельно:
   - equal timestamps;
   - output stamp offset;
   - position stamp offset;
   - duplicate position outputs;
   - monotonicity после delayed velocity и no-controller mode;
   - maximum extrapolation horizon.
5. Нагрузить node потоком выше штатного rate и проверить bounded queue/memory.

**Критерий:** любой timestamp regression должен быть либо явно отброшен/диагностирован, либо обработан документированным способом; нельзя скрыто перепутать event time и publish time.

## 4. Проверить физическую модель и единицы

1. Сгенерировать constant-speed, acceleration, braking, coast, standstill и stop/start сценарии.
2. Сверить путь с аналитическим интегралом.
3. Прогнать km/h, m/s, negative, NaN, infinity, impossible notch и extreme wheel-scale inputs.
4. Проверить slip cases:
   - front-only;
   - rear-only;
   - common-mode;
   - alternating;
   - dropout одного канала;
   - stale command.
5. Проверить, что common-mode disagreement не превращается в ложную уверенность.
6. Для covariance/UKF проверить finite state, PSD/PD covariance, NIS gates и recovery after rejected measurement.

## 5. Проверить карту, геодезию и frames

1. Проверить `ring.csv`: strict `s`, range `[0, ring_len)`, closure, duplicate points и finite columns.
2. Сверить seam interpolation с исходной геометрией и проверить переход через `0/ring_len`.
3. Проверить WGS84/UTM/MGRS/MKRS/ENU на независимом reference implementation.
4. Проверить `map`, `base_link`, antenna lever arm, rover baseline, z/up sign, x/y convention.
5. Прогнать initial snap с master-only, rover-only, dual antenna, opposite direction и out-of-map points.
6. Прогнать ring wrap на нескольких lap и near-lap positions.
7. Проверить, что map matching не заменяет интегрированный arc без отдельного принятого решения.

## 6. Проверить GNSS semantics

1. Стартовое окно: valid fix, no-fix, NaN, delayed fix, one antenna, two antennas, late queue.
2. Проверить, что origin задаётся первым valid fix и не плавает из-за поздней антенны.
3. Mid-route correction: min time, min distance, baseline, cross-track, along-track, burst median и correction-once semantics.
4. Убедиться, что отсутствие GNSS после окна не останавливает normal odometry.
5. Записать отдельно reference-only GNSS processing и estimator GNSS correction; не смешивать их в одном metric.

## 7. Проверить assets и provenance

1. Для каждого asset хранить SHA-256 и путь.
2. Отдельно фиксировать asset commit, runtime commit и metrics commit.
3. Запрещать duplicate/malformed CSV, non-finite values, wrong row length, invalid map range и missing table rows.
4. Запускать provenance tests только в Git checkout, а source-only archive сопровождать инструкцией о том, что Git-dependent checks требуют checkout.
5. Не обновлять historical metric fields так, будто они были заново измерены.

## 8. Независимо перепроверить метрики

Для каждого запуска сохранять JSON record:

```text
commit
runtime_tree_commit
assets_sha256
params_sha256
bag_sha256
command
ros_distro
container/image digest
start/end time
sample counts
velocity RMSE/MAE/p95
position along/cross/3D RMSE/MAE/p95
latency p50/p95/p99/max
CPU/RAM peak
```

Разделять:

- official organiser score;
- offline proxy checker;
- synthetic regression;
- historical results;
- current-head replay.

Только current-head replay с полным provenance можно использовать как claim о текущей версии.

## 9. Sanitizers и quality gates

В ROS/CI toolchain повторить:

- ASAN + UBSAN для core, node и stress test;
- `-Wall -Wextra -Wpedantic -Wconversion` там, где совместимо;
- clang-tidy/static analyzer;
- Python `compileall`, unit tests и shellcheck-equivalent;
- clean install/run test без рабочего каталога в `PYTHONPATH`.

Сохранить не только pass/fail, но и предупреждения, skipped tests и причину каждого skip.

## 10. Release gate

Перед выпуском:

- clean source tree без generated build artifacts;
- нет локального remote `C:\RailBreak` или другого developer-only path;
- archive contains source, docs, assets, tests and checksums;
- archive extracts into a fresh directory;
- corrected archive SHA-256 recorded in `RELEASE_CHECKSUMS.txt`;
- smoke test from extracted directory;
- explicit list of checks not possible without ROS/Docker/rosbag;
- no overstated safety/production/optimality claims.

## Рекомендуемый порядок для следующего ИИ

1. Распаковать архив и прочитать `AUDIT_REPORT.md`.
2. Проверить `ARCHIVE_README.md` и SHA-256.
3. Сначала выполнить standalone core tests.
4. Затем clean ROS build/test.
5. Затем synthetic timing/fault campaign.
6. Затем real rosbag replay и provenance record.
7. Только после этого менять algorithmic parameters.
8. Для каждой правки добавить regression test и повторить весь release gate.
