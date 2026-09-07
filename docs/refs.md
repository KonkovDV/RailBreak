# Источники и границы их применимости

Редакция аудита PR #5, 07.09.2026. Ссылка на статью, стандарт или паспорт
**не является доказательством** корректности кода, калибровки либо соответствия
RailBreak требованиям безопасности. Ниже отдельно указаны математический
контекст, нормативные ориентиры и проверяемые артефакты репозитория.

В этой порции проверены исходники, независимые формулы и regression tests.
Внешние первоисточники из прежней библиографии не перечитывались целиком;
ссылки ниже не помечаются как вновь проверенные. Перед внешним докладом
нужно сверить авторов, название, редакцию, раздел и условия применимости.

## Математический контекст

| Источник / тема | Связь с реализацией | Что из этого не следует |
| --- | --- | --- |
| Julier, scaled UT (2002); Luo & Moroz (2009) | Веса UT и достаточная политика Wc ≥ 0; формулы и независимые тесты в репозитории | Политика не необходима для PSD каждой функции; независимость окна от L относится к kappa=0 |
| Arasaratnam & Haykin, IEEE TSP (2009) | Опциональные 2L cubature-точки | Флаг не делает весь проект отдельным валидированным CKF-продуктом |
| Higham, численная положительная определённость | Контекст ремонта covariance | `project_pd` — Jacobi/floor/проверка Cholesky, не заявленная реализация nearest-SPD оптимизации |
| Kulikova & Kulikov, IFAC (2020); [указанный PDF](https://ifatwww.et.uni-magdeburg.de/ifac2020/media/pdfs/0536.pdf) | Вопросы square-root реализаций и устойчивости | Положительные UT-веса не устраняют posterior-вычитание KSKᵀ; ими нельзя доказать ненужность downdate или универсальное превосходство обычного UKF |
| Bar-Shalom, Li, Kirubarajan (2001) | Контекст NIS/NEES и проверки согласованности | NEES на зависимом twin не доказывает полевую calibration; нужны корректные P, GT, выборка и exposure |
| [arXiv:2512.18508](https://arxiv.org/pdf/2512.18508), ссылка прежнего списка | Кандидат для изучения selection/gating effects | В нашем фильтре R адаптируется по данным: нельзя автоматически объявить NIS обычным или просто усечённым chi-square |
| Davis, сопротивление A+Bv+Cv² | Подписанная продольная сила и размерности | Малоскоростная dead zone не является моделью статического удержания |
| Polach, Wear (2005), формулы (4), (9), (11) | Приближённый 1D контакт в Python generator, не в online UKF | Оценённые a/b/c11 не равны измеренному Hertz-контакту; это не CONTACT/FASTSIM и не полная модель WSP |
| Masreliez (1975); Agamennoni et al. (2012) | Контекст robust updates и ограничения влияния выбросов | Huber и adaptive R сами по себе не дают доказанной статистической нормировки |

Точные контракты этой реализации: [math.md](math.md),
[estimator-priors.md](estimator-priors.md), [integrity-risk.md](integrity-risk.md).
В частности, наличие Cholesky при генерации sigma points не делает фильтр SR-UKF.

## Нормативные ориентиры, не сертификаты

| Источник | Корректная роль здесь |
| --- | --- |
| [UNISIG SUBSET-041 v3.2.0](https://www.era.europa.eu/system/files/2023-01/sos3_index014_-_subset-041_v320.pdf) | Источник, на который прежний текст ссылался для формы ±(5 м + 5%·s). В коде online AL использует оценённый s, checker — GT; форма порога не устанавливает применимость ETCS к данному трамваю или выполнение всех требований |
| [ION: Autonomous Integrity Monitoring Proposal for Critical Rail Applications](https://www.ion.org/publications/abstract.cfm?articleID=12944) | Пример контекста integrity/risk allocation. Настроенные k_sigma/k_over не выведены здесь из подтверждённого системного risk budget; им нельзя приписывать SIL-4/THR по одной ссылке |
| EN 15595:2018 (+A1:2023), UIC 541-05 (2016), как указано в прежнем списке | Кандидаты для проверки требований WSP. 0.4 с в generator — параметр модели; его происхождение/применимость требуют проверки по конкретному пункту и условиям стандарта |
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
- Ссылки прежнего списка на RAIB 08/2026, IR1/2025, Zhu et al. (Wear 2019),
  WILAC и contaminated film требуют проверки по первичным документам до
  внешнего использования чисел/выводов. `mu=0.06` в синтетике — выбранный
  сценарий загрязнённого контакта, не универсальная модель дождя.

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
| [issue #3](https://github.com/KonkovDV/RailBreak/issues/3) и [PR #5](https://github.com/KonkovDV/RailBreak/pull/5) | Текущая работа и обсуждение; запрос review не равен выполненному review |

Числа количества assertions не перенесены из старого списка как вечные:
состав тестов меняется, актуальные результаты привязаны к commit в verification.
Исторические интерпретации доступны в истории Git и review-документах;
они не переиздаются здесь как актуальные технические гарантии.
