# RailBreak — независимый аудит и исправления

Дата: 2026-10-02  
Исходный архив: `RailBreak.git.zip`  
Исходный SHA-256: `e71303fccde3122b72f6d532ef7ed3edf7609df3eca737833fd273b631e4faca`  
Проверенный исходный HEAD: `c6a8c140435abad5b4c3dd2b736e135958c8e716`

## Итог

Репозиторий был проверен как исходное дерево RailBreak, а не только по заявленным метрикам. Проверены документация, ROS-пакет резервной одометрии, standalone/core-ядро, вспомогательные Python-инструменты, assets, provenance-контракты, shell-проверки и доступные регрессионные тесты.

В исправленной версии закрыты два подтверждённых класса дефектов:

1. **Разрыв интерполяции на шве кольцевой карты.** Последний отсчёт `ring.csv` находится немного раньше `ring_len_m`. Ранее `TrackMap::at()` удерживал последнее значение до конца кольца, а затем скачком переходил к первому. Теперь этот короткий участок линейно интерполируется от последнего отсчёта к первому.
2. **Недостаточная защита загрузчика assets.** Загрузчик теперь проверяет форму строк `notch.csv` до индексации, отклоняет дубликаты notch, проверяет допустимый диапазон `s` карты и монотонность таблицы скоростей. Ошибочный asset теперь приводит к явной ошибке конфигурации, а не к тихому перезаписыванию строки или потенциальному выходу за границы.

Также устранены предупреждения о неявной инициализации номера остановки в core-тесте и добавлен regression-тест кольцевого шва.

## Что проверено

- Git refs, ветви, теги, история и diff submission/main.
- Статическая инвентаризация файлов и поиск опасных маркеров.
- `CMakeLists.txt`, `package.xml`, launch-файл, параметры и package-manifest.
- ROS API по исходникам: входы тележек/команды, optional GNSS, выходы скорости/позиции/diagnostics, frame semantics и режим без controller message.
- Core: timestamp regression, stale/outlier/impossible inputs, common-mode slip, ZUPT, GNSS start/correction, order restoration, interpolation/resampling, covariance guards, map wrapping, WGS84/UTM/Moscow frames, wheel scaling и integrity transitions.
- Python validation suite, документационные и metric contracts.
- Git-зависимые provenance/package-manifest тесты в обычном clone.
- Реальные assets из `railbreak_backup_odometry/assets` и ручную сборку standalone core.

## Результаты после исправления

- `railbreak_backup_odometry/tools/test_core.cpp`: проходит, включая новые проверки шва карты.
- `python3 -m compileall -q tools scripts`: проходит.
- `python3 tools/eval/test_docs.py`: проходит.
- `python3 tools/eval/test_metric_contracts.py`: проходит.
- Ранее в исходной проверке также прошли 100 Python eval-тестов, phase-0/T0, plant/contact/metric contracts, synth, organizer, jury и основные C++ standalone targets.
- Provenance и package-manifest тесты прошли в `/data/RailBreak_clone` для исходного Git checkout.

## Ограничения проверки

- В sandbox отсутствуют `cmake` и `ctest`, поэтому package-level CMake/colcon build не был выполнен. Проверка core выполнена прямой сборкой `g++` с теми же include/source контрактами.
- ROS 2 Humble, `tram_vehicle_msgs`, DDS и rosbag2 runtime в sandbox не доступны; end-to-end ROS replay, executor stress и реальные QoS/latency measurements требуют отдельного ROS/Docker окружения.
- ASAN/UBSAN-бинарь был собран, но запуск остановился до теста из-за несовместимого отсутствующего runtime `libasan.so.6`. Это не следует считать успешным sanitizer run; его нужно повторить в ROS/CI toolchain с установленным runtime.
- Числа из старого `MANIFEST.json`, `result.json`, screenshots и `docs/solution/results.md` не приняты автоматически как доказательство текущего HEAD. Сам manifest явно помечает опубликованные metrics как исторические и недействительные для текущей головы.
- Официальное качество на организаторском rosbag нельзя объявлять воспроизведённым без самого rosbag и ROS runtime. В отчёте не заявляется production readiness, safety certification или статистическая гарантия.

## Приоритетные оставшиеся риски

1. Повторить полноценный `colcon build/test` в чистом ROS 2 Humble workspace с включённым и выключенным `DriverControllerCommand`.
2. Прогнать rosbag replay и записать фактические частоты, latency p50/p95/p99, peak CPU/RAM, timestamp monotonicity и output frame.
3. Повторно вычислить velocity/position metrics на фиксированном held-out наборе, сохранив commit, assets, params, bag hash, command и environment.
4. Повторить ASAN/UBSAN/clang-tidy или эквивалентную проверку в том же toolchain, который используется CI.
5. Проверить границы GNSS correction на реальном времени и убедиться, что correction не нарушает relative-only режим при отсутствии GNSS.

## Состав исправленной поставки

- Исходное дерево RailBreak без bare-only структуры.
- `AUDIT_REPORT.md` — этот отчёт.
- `AI_WORKPLAN.md` — пошаговый план для следующего ИИ/инженера.
- `ARCHIVE_README.md` — как распаковать и какие проверки запускать.
- `RELEASE_CHECKSUMS.txt` — manifest SHA-256 исходного дерева и ссылка на внешний checksum итогового архива. Итоговый архивный SHA-256 также передаётся в соседнем `.sha256` файле.
