# Проверки и граница доказанного

Редакция `tramDR-0.0.11`. Аудит читал `main`
`9e65dc1fde96c419a437ae5be62ad37c12e34495`. Строка ниже про CI
`9cd215e` — прогон того коммита, не «текущий main».
«Тест прошёл» означает выполнение
конкретного набора проверок на конкретной версии, а не проверку всех
режимов вагона или сертификацию. `kModelVersion` в
`tram_dr_localization/include/tram_dr_localization/types.hpp` — версия модели,
не уникальный идентификатор сборки: для evidence нужен также commit.

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
| `test_hackathon_phase0` | `brake_source`, combined notch, радиусный prior, interval monitor, SCA extremal |

Всего девять зарегистрированных наборов, а не девять отдельных assertions.
 утро: `ctest -C Release` в `standalone/build` — 9/9.
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

## 3. Исторические regression-свидетельства

Baseline: `91e20a74f0b526fc782b4eb7ddfd3afff73d575b`.
Численная порция: `ec5f5cd28174e191cb8f3ac39e191bd300d1d08e`.
Физическая C++ порция: `dde3bc7d6594c4a5378348940271170ac7017561`.
Исправленное расписание prior с устранёнными ошибками переноса файла:
`5bd39a41aeadcabb80b057fe908fa49cdbe51f44`.
HMI без экспозиции: `e36358a669d098f37f7de07930d0723021e82a11`.
Scorer coverage/join: `0827dcdd564df226c7f9a4eeeb8e8eae36e73666`.
Python physics: `faa4faebd0b3116f7584b8716d971cbf6cdf97e4`.

| Проверка | До | После | Где |
| --- | --- | --- | --- |
| Numerical edges | 119 checks, 76 failures | 119 checks, 0 failures | GCC 11.5, C++17, Release/NDEBUG |
| Plant contracts | 29 checks, 7 failures | 29 checks, 0 failures | GCC 11.5, Release; повторено в Debug |
| Prior scheduling | CI cpp/asan failure на test-only commit `6327295` | CI cpp/asan success на `5bd39a4` | Полный UKF, assertions не ослаблены |
| HMI exposure | 10 tests, 7 failures | 10 tests, 0 failures | Python 3.13, затем Python CI |
| Scorer exposure/join | 13 tests, 17 failures/subtests | 13 tests, 0 failures | Python 3.13, затем Python CI |
| Python physics | 13 tests, 17 failures/subtests | 13 tests, 0 failures | Python 3.13, затем Python CI |

Failure — проваленная проверка, **не** обязательно отдельный дефект.
Python subtests позволяют получить несколько failures в одном методе.
Численный тест включает независимый long-double LDL, матрицы размерности
1…16, нулевые/неопределённые/плохо обусловленные случаи, healthy no-op,
нечисловые входы и beta до DBL_MAX. C++ plant-тест проверяет оба направления
выбега и не блокирует разворот от внешних сил.

Python physics-тесты отдельно проверяют None/пустой буфер/постоянную память,
переменный dt, stop event в обоих направлениях, инерционную массу и внешние
силы. В одном analytic witness контакт намеренно заменён линейной силой,
а настоящий Newton solver сопоставлен с закрытой формой. Другие fixtures
используют реальную нелинейную контактную функцию. Снижение числа вычислений
силы с 85 до 6 в указанных линейных случаях — не замер WCET или точности
вагона. Формулы, параметры witness и ограничения: [math.md](math.md), §8.

Для prior фиксируются результат job и переход red→green; stdout с точным
числом его проверок отдельно не извлечён. Нельзя выдавать предполагаемую
строку лога за наблюдавшийся результат. При переносе полного UKF-файла в
draft-ветке возникли дополнительные ошибки транскрипции; diff review их
выявил и последующие commits устранили. Итоговый diff UKF относительно
baseline — только перенос вызова prior и поясняющий комментарий.

Все четыре job (cpp, asan, python, ros) подтверждены на Python-fix `faa4faeb`
и затем на docs-checkpoint `908ca160`:
[PR run](https://github.com/KonkovDV/RailBreak/actions/runs/34158342493),
[push run](https://github.com/KonkovDV/RailBreak/actions/runs/34158338834).
Статусы более ранних commits не являются подтверждением последнего HEAD;
после следующих изменений checks проверяются заново.

Локальная среда аудита не имела CMake, ROS, Docker/colcon и пригодного
ASan runtime. Локально запускались перечисленные C++ binaries, 23 Python
контракта метрик, 13 контрактов физики и 7 фикстур структурного checker
документации (включая refs). Эти 7 фикстур проверяют сам checker, не реальное
дерево репозитория. Полный core/e2e, существующие Python suites, ROS,
санитайзеры и документация реального дерева проверялись в CI.
Недоступность runtime не считается успешным sanitizer-прогоном.
Локальные исходники/зависимости сверены с Git blob hashes репозитория.
Тесты метрик создают небольшие записи и вызывают настоящие checker/scorer
и их зависимости; UKF они не запускают. Намеренная подмена линейного контакта
в physics witness описана отдельно и не выдаётся за физическую GT-валидацию.

### Новый checkpoint

`82ebf744d6237c2eebf6a2074254a6ee288240b0` = новый baseline `38f65c1` +
документальная OSINT-порция. Все четыре job успешны в обоих runs:
[PR #6](https://github.com/KonkovDV/RailBreak/actions/runs/34199641322),
[push](https://github.com/KonkovDV/RailBreak/actions/runs/34199455386).
Это свежая проверка дерева после изменений владельца, не перенос зелёного
статуса PR #5. CI по-прежнему не запускает полноценный ROS runtime graph.
Локально повторены 13 physics + 23 metric tests. Состав HMI-таблиц проверен
отдельно как арифметика документа; новая таблица RMSE не измерялась.

После высадки на `main`: contact-force `907f40b`/`9d6dd9c`, docs/PPTX
`9cd215e`. Все четыре job успешны:
[push `9cd215e`](https://github.com/KonkovDV/RailBreak/actions/runs/34204912446).
Последующие commits требуют своих checks.

## 4. Python и synthetic e2e

Python 3.11+, основным скриптам достаточно стандартной библиотеки:

```bash
python3 tools/eval/test_eval.py
python3 tools/eval/test_metric_contracts.py
python3 tools/eval/test_python_plant_contracts.py
python3 tools/eval/test_contact_contracts.py
python3 tools/synth/test_generate.py
python3 tools/eval/no_gnss_scan.py
python3 tools/eval/test_docs.py
python3 tools/synth/generate.py --out synth/runs
python3 tools/eval/run_e2e.py --ukf standalone/build/replay_ukf
python3 tools/synth/score.py --runs synth/runs
python3 tools/eval/test_eval.py ReplayCatchupTests
```

CI выполняет эти команды. 23 metric tests покрывают HMI без экспозиции,
исходные знаменатели coverage, повтор GT, неупорядоченные/нечисловые времена,
пустые результаты, граничный tolerance и независимый brute-force join.
13 physics tests и 8 contact tests проверяют выбранные контракты Python-twin.
Это не 36 новых сценариев движения и не новая таблица точности вагона.
Структурный docs checker обходит `CURRENT_DOCS` в
`tools/eval/test_docs.py` (12 файлов на , включая refs.md,
t0-runbook и data-contract): относительные ссылки, некоторые пути
исходников, code fences, версии и список CTest. Он не проверяет внешние
URL, heading anchors, LaTeX или истинность текста.

`mismatch_r0` является предусмотренным отрицательным контролем и исключением
из общего fail gate e2e; зелёный job не означает HMI = 0 во всех сценариях.
См. [checker.md](checker.md).

[metrics.md](metrics.md) публикует числа для `tramDR-0.0.11` (seed 42).
В основной HMI-таблице показан ноль у **18/19** сценариев; четыре отдельных
route-сценария дают ноль, итого **22/23** в двух таблицах.
`mismatch_r0` — **0.515**. Это пересчёт строк документа, не независимое
подтверждение опубликованных траекторий. Ноль условной HMI при трёх
OK-кадрах нельзя выдавать за подтверждение малой вероятности риска.
На `slide_*` заявлены небольшие изменения DEGRADED/LOST относительно 0.0.10
при RMSE в прежнем округлении; это не доказанное улучшение фильтра.

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
 на этом Windows-хосте граф запускался в `railbreak-tram_dr`
(Humble desktop): `live.launch.py` и `replay.launch.py` с синтетическим
bag. Лог и граница: [ros-rehearsal.md](ros-rehearsal.md). `colcon test`
в том контейнере утром  не повторялся — только `colcon build`.

## 6. Открытая матрица испытаний

| Область | Следующая проверка / ограничение |
| --- | --- |
| Multirate | Время датчика против filter dt; ring/freeze/omega-dot/hold/quantization |
| ROS input | В новом исходнике подтверждено отбрасывание NaN brake; нужны регрессии invalidation/recovery, stale/reordered и leftover catch-up |
| ROS output | Identity quaternion и \(P=10^6\) на неоцениваемых осях подтверждены и в исходнике, и одним echo . После стопа `/clock` возраст растёт по `clock_stall_s` (не шаг фильтра). Полевой bag не прогонялся |
| Bag/метрики | Интерполяция, экстраполяция и coverage bag-пути; time-weighted exposure; strict JSON и malformed input |
| Статистика | Нормированность NIS/NEES, false alarms, exposure, независимые поездки |
| Физика | Полевые m/r0/тяга/Дэвис/уклон; статическое удержание, WSP и joint faults |
| Python generator | Полная валидация входов, clipping/корни и coupled-ODE convergence; не весь solver domain покрыт fixtures |
| Источники | Выборочная проверка первоисточников выполнена; точный объём и непрочитанное — [OSINT/triage](review/osint-triage.md), применимость к вагону остаётся открытой |
| Презентация | PPTX пересобран из `pitch.md` (`9cd215e`); визуальную вёрстку всё равно сверить на проекторе |
| Производительность | WCET и latency на целевом контроллере, DDS/executor, HIL |
| Safety | Независимый hazard analysis и действия потребителя, не только PL/HMI |

Не все исходники/режимы имеют индивидуальные regression-тесты. Реальные
записи заказчика, его контроллер и HIL здесь недоступны. Нельзя формулировать
результат как «всё протестировано» или «готово к эксплуатации».

Исторические audit/review-файлы сохраняют происхождение находок; актуальные
контракты — [math.md](math.md), [architecture.md](architecture.md),
[estimator-priors.md](estimator-priors.md), [integrity-risk.md](integrity-risk.md).
