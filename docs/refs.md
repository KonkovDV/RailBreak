<p align="center"><img src="../Logo.png" alt="RailBreak" width="280"></p>

# Источники и границы их применимости

Редакция `tramDR-0.0.11`. Каждая ссылка ниже стоит рядом с тем, что именно она
даёт реализации, и с границей переноса. Корректность кода доказывают не ссылки,
а тесты репозитория — они собраны в последнем разделе.

Сверены с первоисточником: Polach, фрагмент UNISIG SUBSET-041, каталог BSI,
официальное резюме RAIB и сайт хакатона. Остальные записи прежнего каталога —
кандидаты, отмеченные как таковые.

## Математический контекст

| Источник / тема | Связь с реализацией | Граница переноса |
| --- | --- | --- |
| Julier, scaled UT (2002); Luo & Moroz (2009) | Веса UT и достаточная политика Wc ≥ 0; формулы и независимые тесты в репозитории | Политика достаточная, не необходимая; независимость окна от L — при kappa=0 |
| Arasaratnam & Haykin, IEEE Trans. Autom. Control 54(6), 2009 | Опциональные 2L cubature-точки | Режим ядра, не отдельный продукт CKF |
| Higham, численная положительная определённость | Контекст ремонта covariance | `project_pd` — Jacobi/floor/проверка Cholesky; задача nearest-SPD не решается |
| Kulikova & Kulikov, IFAC (2020); [указанный PDF](https://ifatwww.et.uni-magdeburg.de/ifac2020/media/pdfs/0536.pdf) | Вопросы square-root реализаций и устойчивости | Положительные UT-веса защищают prediction; posterior-вычитание KSKᵀ защищает `project_pd` |
| Bar-Shalom, Li, Kirubarajan (2001) | Контекст NIS/NEES и проверки согласованности | Полевая calibration требует независимого GT, выборки и exposure; NEES на twin — согласованность модели |
| [arXiv:2512.18508](https://arxiv.org/abs/2512.18508), Barak Or | Сверен abstract: selection/gating | Результаты для linear-Gaussian; распределение NIS при adaptive-R нужно мерить отдельно |
| Davis, сопротивление A+Bv+Cv² | Подписанная продольная сила и размерности | Малоскоростная dead zone; стоянку ведёт ZUPT |
| [O. Polach, Wear 258 (2005), 992–1000](https://www.oldrich.polach.ch/data/object_4/Wear_2005.pdf), DOI 10.1016/j.wear.2004.03.046 | Сверены (4), (9), (11), (12), §2.5 и таблицы параметров; приближённый 1D контакт Python generator, не online UKF | Оценённые a/b/c11 и типовые параметры — не паспорт вагона; CONTACT/FASTSIM, полный 3D Polach и WSP вне модели |
| Masreliez (1975); Agamennoni et al. (2012) | Контекст robust updates и ограничения влияния выбросов | Huber и adaptive R ограничивают влияние выбросов; статистическую нормировку NIS они не задают |

Точные контракты этой реализации: [math.md](math.md),
[estimator-priors.md](estimator-priors.md), [integrity-risk.md](integrity-risk.md).
Фильтр — ковариационный UKF: Cholesky строит sigma points, P хранится явно.

## Нормативные ориентиры

Стандарты ниже — линейка величин и контекст требований; сертификатом они не служат.

| Источник | Роль здесь |
| --- | --- |
| [UNISIG SUBSET-041 v3.2.0](https://www.era.europa.eu/system/files/2023-01/sos3_index014_-_subset-041_v320.pdf), §5.3.1.1 | Подтверждена форма ±(5 м + 5%·s) для measured distance от reference point, включая требование safe confidence interval при malfunction. Online AL использует оценённый s, checker — GT. Используется как линейка величин: трамвай вне ETCS |
| [ION: Autonomous Integrity Monitoring Proposal for Critical Rail Applications](https://www.ion.org/publications/abstract.cfm?articleID=12944) | Контекст integrity/risk allocation. k_sigma/k_over настроены эмпирически; вывод из системного risk budget и SIL/THR — задача safety case |
| [BSI EN 15595:2018+A1:2023](https://knowledge.bsigroup.com/products/railway-applications-braking-wheel-slide-protection-2); UIC 541-05 (2016) | Каталог BSI подтверждает редакцию и дату. 0.4 с — параметр generator; требования WSP и UIC — контекст |
| EN 13452; ГОСТ 8802; ПТЭ трамвая | Контекст применимых требований |

Нулевой HMI на синтетике, SPD и зелёный CI подтверждают корректность реализации;
SIL, THR и безопасность маршрута устанавливает safety case. См.
[integrity-risk.md](integrity-risk.md).

## Подвижной состав, маршрут и синтетика

- [ПК ТС: «Львёнок»](https://pk-ts.org/produkciya/l-venok/) и
  [TransPhoto 71-911ЕМ-03](https://transphoto.org/page/671/) — открытые описания
  модели вагона. Масса, диаметр и мощность для калибровки берутся из паспорта
  точной модификации.
- `vehicle_lvenok_moscow.yaml`, `vehicle_vityaz_m.yaml` и
  `vehicle_combino_nf100.yaml` — конфигурации модели: m0/r0 в них — номинал
  профиля, а не паспорт; московский вагон и twin ведутся раздельно.
- Геометрия `route_10.yaml` — полилиния остановок OSM для сценариев. Условия
  ODbL — в [NOTICE](../NOTICE). Координатными измерениями UKF остановки
  не служат. Ось маршрута для сдачи построена по RTK `train` (`assets/`).
- Профили уклона `*route10*` в generator — сценарные оценки i(s); online-фильтр
  их не читает.
- [RAIB 08/2026](https://www.gov.uk/raib-reports/report-08-slash-2026-collision-between-two-passenger-trains-near-talerddig):
  сверено официальное резюме. Авария heavy rail при сочетании сцепления,
  работы песочниц, скорости и защиты от проезда — мотивация сценария низкого
  сцепления. IR1/2025, Zhu et al. (Wear 2019), WILAC и contaminated film —
  кандидаты для следующих сценариев. `mu=0.06` — выбранный сценарий мокрого рельса.

Полные записи и граница заимствования для сдаваемого фильтра — в [solution/references.md](solution/references.md).
Hasberg, Hensel, Stiller, IEEE T-ITS, 2012, DOI 10.1109/TITS.2011.2177522: состояние рельсового
экипажа — одномерная координата вдоль оси пути, а не свободный плоский фильтр.
Эта постановка лежит в основе сдачи; их probabilistic curvemap — кандидат.
von Einem, Cramariuc, Siegwart, Cadena, Tschopp, IEEE ITSC 2023, Bilbao,
DOI 10.1109/ITSC57777.2023.10422075: на трамвае Цюриха статья сообщает 4.78 м RMSE,
локальное движение там — visual-inertial odometry, вне входного контракта хакатона.
CLUG2 (ION 2024) и EGNSS MATE (2026) добавляют IMU и бализы — датчики вне
контракта. Перенесено главное: одометрия вдоль заранее построенной оси
и точечные якоря (остановки играют роль бализ). Palmer &
Nourani-Vatani, IROS 2018, DOI 10.1109/IROS.2018.8594473, arXiv:1803.02237, оценивают диаметр колеса как состояние EKF
по независимому датчику скорости; без доплера масштаб наблюдаем по карте —
так RailBreak и снимает `k₀`, а не по каталожному радиусу.

Другие кандидаты из прежней библиографии: Willsky & Jones (1976), Isermann
(2006), Palmer & Nourani-Vatani (2018), Malvezzi/Allotta/Rinchi (2011), Allotta
et al. (2002), Lauer & Stein (2015), Hasberg/Hensel/Stiller (2012), Brocard
et al. (2020), Kim et al. (2015), Kuhse et al. (2023), Bédard et al., Zhang
et al. (CMES 2025), Shrestha (2025), Reid et al. (2019). Это каталог кандидатов
для чтения. Полный GLR, inertial witness и moment-based adhesion observer
в ядро не входят; p99/WCET целевого ROS-графа меряется на целевой машине.

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
| [CHANGELOG.md](../CHANGELOG.md) | Где лежит сдаваемый пакет и версия исследовательского ядра |
| [HACKATHON.md](../HACKATHON.md) | Состав сдачи |

Число проверок привязано к commit в [verification.md](verification.md):
состав тестов растёт вместе с кодом.
