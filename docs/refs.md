# Источники и границы их применимости

Редакция `tramDR-0.0.11`, 08.09.2026. Ссылка на статью, стандарт или паспорт
**не является доказательством** корректности кода, калибровки либо соответствия
RailBreak требованиям безопасности. Ниже отдельно указаны математический
контекст, нормативные ориентиры и проверяемые артефакты репозитория.

07.09 проверялись исходники, независимые формулы и regression tests.
08.09 выборочно сверены внешние первоисточники: Polach, фрагмент UNISIG,
каталог BSI, официальное резюме RAIB и сайт хакатона. Точный объём чтения,
недоступные документы и triage — [OSINT 08.09](review/osint-triage-2026-09-08.md).
Весь прежний каталог не объявляется верифицированным.

## Математический контекст

| Источник / тема | Связь с реализацией | Что из этого не следует |
| --- | --- | --- |
| Julier, scaled UT (2002); Luo & Moroz (2009) | Веса UT и достаточная политика Wc ≥ 0; формулы и независимые тесты в репозитории | Политика не необходима для PSD каждой функции; независимость окна от L относится к kappa=0 |
| Arasaratnam & Haykin, IEEE TSP (2009) | Опциональные 2L cubature-точки | Флаг не делает весь проект отдельным валидированным CKF-продуктом |
| Higham, численная положительная определённость | Контекст ремонта covariance | `project_pd` — Jacobi/floor/проверка Cholesky, не заявленная реализация nearest-SPD оптимизации |
| Kulikova & Kulikov, IFAC (2020); [указанный PDF](https://ifatwww.et.uni-magdeburg.de/ifac2020/media/pdfs/0536.pdf) | Вопросы square-root реализаций и устойчивости | Положительные UT-веса не устраняют posterior-вычитание KSKᵀ; ими нельзя доказать ненужность downdate или универсальное превосходство обычного UKF |
| Bar-Shalom, Li, Kirubarajan (2001) | Контекст NIS/NEES и проверки согласованности | NEES на зависимом twin не доказывает полевую calibration; нужны корректные P, GT, выборка и exposure |
| [arXiv:2512.18508](https://arxiv.org/abs/2512.18508), Barak Or | В поисковом abstract подтверждена тема selection/gating; полная загрузка не удалась | Linear-Gaussian результаты не разрешают автоматически объявить NIS нашего adaptive-R фильтра обычным или просто усечённым chi-square |
| Davis, сопротивление A+Bv+Cv² | Подписанная продольная сила и размерности | Малоскоростная dead zone не является моделью статического удержания |
| [O. Polach, Wear 258 (2005), 992–1000](https://www.oldrich.polach.ch/data/object_4/Wear_2005.pdf), DOI 10.1016/j.wear.2004.03.046 | Сверены (4), (9), (11), (12), §2.5 и таблицы параметров; приближённый 1D контакт Python generator, не online UKF | Оценённые a/b/c11 не равны измеренному Hertz-контакту; типовые параметры не паспорт вагона. Это не CONTACT/FASTSIM, полный 3D Polach или полная модель WSP |
| Masreliez (1975); Agamennoni et al. (2012) | Контекст robust updates и ограничения влияния выбросов | Huber и adaptive R сами по себе не дают доказанной статистической нормировки |

Точные контракты этой реализации: [math.md](math.md),
[estimator-priors.md](estimator-priors.md), [integrity-risk.md](integrity-risk.md).
В частности, наличие Cholesky при генерации sigma points не делает фильтр SR-UKF.

## Нормативные ориентиры, не сертификаты

| Источник | Корректная роль здесь |
| --- | --- |
| [UNISIG SUBSET-041 v3.2.0](https://www.era.europa.eu/system/files/2023-01/sos3_index014_-_subset-041_v320.pdf), §5.3.1.1 | Подтверждена форма ±(5 м + 5%·s) для measured distance от reference point, включая требование safe confidence interval при malfunction. Online AL использует оценённый s, checker — GT; это не эквивалентная реализация всех условий и не установление применимости ETCS к трамваю |
| [ION: Autonomous Integrity Monitoring Proposal for Critical Rail Applications](https://www.ion.org/publications/abstract.cfm?articleID=12944) | Пример контекста integrity/risk allocation. Настроенные k_sigma/k_over не выведены здесь из подтверждённого системного risk budget; им нельзя приписывать SIL-4/THR по одной ссылке |
| [BSI EN 15595:2018+A1:2023](https://knowledge.bsigroup.com/products/railway-applications-braking-wheel-slide-protection-2); UIC 541-05 (2016) | Каталог BSI подтверждает редакцию/дату, не конкретный пункт 0.4 с. Полные требования WSP и UIC здесь не сверены. 0.4 с остаётся параметром generator, не доказательством соответствия |
| EN 13452; ГОСТ 8802; ПТЭ трамвая | Контекст применимых требований. Соответствие реализации не проверено и не заявляется |

Нулевой HMI на синтетике, SPD и зелёный CI не устанавливают SIL, THR или
безопасность маршрута. См. [integrity-risk.md](integrity-risk.md).

## Подвижной состав, маршрут и синтетика

- [ПК ТС: «Львёнок»](https://pk-ts.org/produkciya/l-venok/) и
  [TransPhoto 71-911ЕМ-03](https://transphoto.org/page/671/) — ссылки прежнего
  OSINT-списка, не измерения конкретного вагона. Числа массы, диаметра и мощности
  следует привязать к точной модификации и первичному паспорту.
- `vehicle_lvenok_moscow.yaml`, `vehicle_vityaz_m.yaml` и
  `vehicle_combino_nf100.yaml` — конфигурации модели. Наличие m0/r0 в YAML
  не подтверждает паспортную точность; нельзя смешивать московский вагон и twin.
- Геометрия `route_10.yaml` — пример полилинии остановок OSM, не обследованная
  ось пути. Условия ODbL см. в [NOTICE](../NOTICE). Остановки не являются
  координатными измерениями UKF.
- Профили уклона `*route10*` в generator — сценарные оценки, не обследованный
  профиль i(s) и не вход online-фильтра. Старые новости о маршруте/ремонте
  не подтверждают его текущую геометрию или расписание.
- [RAIB 08/2026](https://www.gov.uk/raib-reports/report-08-slash-2026-collision-between-two-passenger-trains-near-talerddig):
  проверено официальное резюме, не полный отчёт. Авария heavy rail при
  сочетании сцепления, работы песочниц, скорости и защиты от проезда —
  не обследование маршрута 10 и не доказательство свойств нашего детектора.
  IR1/2025, Zhu et al. (Wear 2019), WILAC и contaminated film остаются
  кандидатами для проверки. `mu=0.06` — выбранный сценарий, не универсальная модель дождя.

Другие кандидаты из прежней библиографии: Willsky & Jones (1976), Isermann
(2006), Palmer & Nourani-Vatani (2018), Malvezzi/Allotta/Rinchi (2011), Allotta
et al. (2002), Lauer & Stein (2015), Hasberg/Hensel/Stiller (2012), Brocard
et al. (2020), Kim et al. (2015), Kuhse et al. (2023), Bédard et al., Zhang
et al. (CMES 2025), Shrestha (2025), Reid et al. (2019). Это **не полностью
верифицированный каталог**. В текущем коде нет оснований приписывать себе
полный GLR, независимый inertial witness, moment-based adhesion observer
или измеренный p99/WCET целевого ROS-графа.

## Внутренние свидетельства — отдельно от литературы

| Артефакт | Что проверяется |
| --- | --- |
| [test_ut_weights_psd.cpp](../standalone/test_ut_weights_psd.cpp) | Алгебра UT независимо от заголовков ядра |
| [test_ut_weights_header.cpp](../standalone/test_ut_weights_header.cpp) | Поставляемые helpers против формул |
| [test_numerical_edges.cpp](../standalone/test_numerical_edges.cpp) | Floor/rounding, atomic failure и крайние параметры |
| [test_plant_contracts.cpp](../standalone/test_plant_contracts.cpp) | C++ SI-силы, память и stop event |
| [test_prior_scheduling.cpp](../standalone/test_prior_scheduling.cpp) | Расписание процесса массы в настоящем UKF |
| [test_metric_contracts.py](../tools/eval/test_metric_contracts.py) | HMI/coverage и независимый nearest-GT reference |
| [test_python_plant_contracts.py](../tools/eval/test_python_plant_contracts.py) | Python drive/stop и аналитический Newton witness |
| [verification.md](verification.md) | Наблюдавшиеся результаты, версии, окружение и открытые проверки |
| [CHANGELOG.md](../CHANGELOG.md) | Исторические изменения; старые статусы не заменяют текущий triage |
| [OSINT 08.09](review/osint-triage-2026-09-08.md) | Свежая проверка первоисточников, статусы доказательств и вопросы перед защитой |
| [issue #3](https://github.com/KonkovDV/RailBreak/issues/3) | Исторический реестр находок; актуальный статус сдачи — [`HACKATHON.md`](../HACKATHON.md) |

Числа количества assertions не перенесены из старого списка как вечные:
состав тестов меняется, актуальные результаты привязаны к commit в verification.
Исторические интерпретации доступны в истории Git и review-документах;
они не переиздаются здесь как актуальные технические гарантии.
