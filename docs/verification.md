# Проверки и граница доказанного

Редакция аудита PR #5, 07.09.2026. «Тест прошёл» означает выполнение
конкретного набора проверок на конкретной версии, а не проверку всех
режимов вагона или сертификацию. Актуальные статусы последнего commit
смотрите в [PR #5](https://github.com/KonkovDV/RailBreak/pull/5).

## 1. Воспроизводимая сборка без ROS

Из корня репозитория; нужны CMake ≥ 3.16 и компилятор C++17:

```bash
cmake -S standalone -B standalone/build -DCMAKE_BUILD_TYPE=Release
cmake --build standalone/build --parallel
ctest --test-dir standalone/build --output-on-failure
```

| CTest target | Проверяемая область |
| --- | --- |
| `test_core` | Сквозное поведение UKF, режимы и сценарии ядра |
| `test_integrity_contracts` | Вход/время, freshness, ZUPT, per-channel confidence, latch и численный отказ |
| `test_ut_weights_psd` | Независимая алгебра scaled UT без заголовков ядра |
| `test_ut_weights_header` | Сопоставление поставляемого UT helper с формулами |
| `test_prior_and_nis` | Формулы mass prior и эффект насыщения post-Huber NIS |
| `test_numerical_edges` | PSD repair/floor, atomic failure, крайние UT параметры |
| `test_plant_contracts` | SI-силы, память привода, событие остановки при выбеге |
| `test_prior_scheduling` | Процесс массы внутри UKF при silence, NaN и rest |

Всего восемь зарегистрированных наборов, а не восемь отдельных assertions.
`replay_ukf` — дополнительный executable, не отдельный CTest target.
Новые regression-тесты используют явные проверки и выполняются при NDEBUG.

## 2. Санитайзеры

```bash
cmake -S standalone -B standalone/build-asan \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer"
cmake --build standalone/build-asan --parallel
ASAN_OPTIONS=detect_leaks=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
ctest --test-dir standalone/build-asan --output-on-failure
```

Нужны поддерживаемый компилятор и runtime ASan/UBSan. CI теперь собирает
все targets и запускает CTest, а не вручную перечисленные старые пять
executable. Добавление нового теста в CMake больше не должно молча исключать
его из санитайзеров. ASan/UBSan не заменяют model validation или race testing.

## 3. Фактические regression-свидетельства этой порции

Baseline: `91e20a74f0b526fc782b4eb7ddfd3afff73d575b`.
Численная порция: `ec5f5cd28174e191cb8f3ac39e191bd300d1d08e`.
Физическая порция: `dde3bc7d6594c4a5378348940271170ac7017561`.
Исправленное расписание prior с устранёнными ошибками переноса файла:
`5bd39a41aeadcabb80b057fe908fa49cdbe51f44`.

| Проверка | До | После | Где |
| --- | --- | --- | --- |
| Numerical edges | 119 checks, 76 failures | 119 checks, 0 failures | GCC 11.5, C++17, Release/NDEBUG |
| Plant contracts | 29 checks, 7 failures | 29 checks, 0 failures | GCC 11.5, Release; повторено в Debug |
| Prior scheduling | CI cpp/asan failure на test-only commit `6327295` | CI cpp/asan success на `5bd39a4` | Полный UKF, assertions не ослаблены |

76 и 7 — количества проваленных проверок, **не** число разных дефектов.
Численный тест включает независимый long-double LDL, матрицы размерности
1…16, нулевые/неопределённые/плохо обусловленные случаи, healthy no-op,
нечисловые входы и beta до DBL_MAX. Физический тест проверяет оба направления
выбега и не блокирует разворот от внешних сил.

Для prior фиксируются результат job и переход red→green; stdout с точным
числом его проверок отдельно не извлечён. Нельзя выдавать предполагаемую
строку лога за наблюдавшийся результат. При переносе полного UKF-файла в
draft-ветке возникли дополнительные ошибки транскрипции; diff review их
выявил и последующие commits устранили. Итоговый diff UKF относительно
baseline — только перенос вызова prior и поясняющий комментарий.

Подтверждённый C++/ASan/Python run для `5bd39a4`:
[GitHub Actions](https://github.com/KonkovDV/RailBreak/actions/runs/34153221667).
Статусы более ранних commits не являются подтверждением последнего HEAD.
Окончательный ROS и остальные checks должны проверяться на последнем
commit PR, включая изменения версии/документации.

Локальная среда аудита не имела CMake, ROS, Docker/colcon и пригодного
ASan runtime. Поэтому локально запускались только перечисленные независимые
C++ binaries; полный core/e2e и санитайзеры проверялись в CI. Недоступность
runtime не считается успешным sanitizer-прогоном.

## 4. Python и synthetic e2e

Python 3.11+, основным скриптам достаточно стандартной библиотеки:

```bash
python3 tools/eval/test_eval.py
python3 tools/synth/test_generate.py
python3 tools/eval/no_gnss_scan.py
python3 tools/synth/generate.py --out synth/runs
python3 tools/eval/run_e2e.py --ukf standalone/build/replay_ukf
python3 tools/synth/score.py --runs synth/runs
python3 tools/eval/test_eval.py ReplayCatchupTests
```

CI выполняет эти команды. `mismatch_r0` является предусмотренным
отрицательным контролем и исключением из общего fail gate e2e; зелёный job
не означает HMI = 0 во всех сценариях. См. [checker.md](checker.md).

[metrics.md](metrics.md) содержит **исторические** таблицы 0.0.10.
Эта порция не переименовывает их в измерения изменённого ядра. Новая таблица
должна включать сохранённый output, commit, seed, profile, join/coverage и
метаданные платформы; одного запуска score без извлечённых чисел недостаточно.

## 5. ROS CI и ручное воспроизведение

CI использует `ros:humble-ros-base`, rosdep, colcon build и ament gtests
`test_types`, `test_plant`, `test_sca`, `test_ukf`; компилируются все четыре
ноды. В настроенной ROS 2 Humble среде:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select tram_dr_localization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select tram_dr_localization \
  --event-handlers console_direct+
colcon test-result --all --verbose
```

Сборка нод и library gtests **не** равны запуску реального ROS-графа.
Dockerfile/Compose и launch прочитаны, но их runtime-проверка на локальном
стенде этого аудита не выполнялась.

## 6. Открытая матрица испытаний

| Область | Следующая проверка / ограничение |
| --- | --- |
| Multirate | Время датчика против filter dt; ring/freeze/omega-dot/hold/quantization |
| ROS input | NaN brake, stale flags, параметры, очень малые dt и накопление времени |
| ROS output | Quaternion, frames, неоцениваемые covariance, согласование diagnostics |
| Checker/score | Нулевой знаменатель, исходный coverage, reuse GT и join tolerances |
| Статистика | Нормированность NIS/NEES, false alarms, exposure, независимые поездки |
| Физика | Полевые m/r0/тяга/Дэвис/уклон; статическое удержание, WSP и joint faults |
| Производительность | WCET и latency на целевом контроллере, DDS/executor, HIL |
| Safety | Независимый hazard analysis и действия потребителя, не только PL/HMI |

Не все исходники/режимы имеют индивидуальные regression-тесты. Реальные
записи заказчика, его контроллер и HIL здесь недоступны. Нельзя формулировать
результат как «всё протестировано» или «готово к эксплуатации».

Исторические audit/review-файлы сохраняют происхождение находок; актуальные
контракты — [math.md](math.md), [architecture.md](architecture.md),
[estimator-priors.md](estimator-priors.md), [integrity-risk.md](integrity-risk.md).
