<p align="center"><img src="../../Logo.png" alt="RailBreak" width="280"></p>

# Источники и место RailBreak в литературе 2012–2026

Для каждой работы указано, что из неё взято в ноду и где она стоит в плане развития. Кольцо, остановки, таблица ручки, состояние `[s, v, k, bₐ]`, якорь по ZUPT, перевод км/ч, выход `base_link`, монитор целостности и скореры написаны в этом репозитории. Числа из статей относятся к их данным и приведены для контекста.

## Сводка

| Направление | Работы | В RailBreak |
|---|---|---|
| Одномерное состояние вдоль пути | Hasberg 2012; von Einem 2023; Zhang 2025; Zhang 2026 | Дуга `s` по кольцу, поза из карты |
| Робастное слияние при несогласии датчиков | Palmer 2018; Jang 2026; González 2026 | `R = 25 (м/с)²` при `ν²/S > 16`, общая мода как отдельный режим |
| Сцепление и юз | Pichlík 2021; Bouchama 2021; Namoano 2024; CMES 2025; Zha 2026; ETAE 2026 | Карточка невязок, раздельные отказ измерения и отказ модели, `mu_estimate = null` |
| Независимые каналы | Potokar 2024; Dieckow 2025 | План: третий канал скорости |
| Целостность и калиброванная неопределённость | Maharmeh 2025; Kumar 2025; Hore 2026; Lai, Raskutti 2026 | Эмпирическая граница по `train`, проверка на `val`; план: онлайн-конформная калибровка |
| Нормативные линейки | SUBSET-041; ГКИНП (ОНТА)-01-268-02 | Масштаб ±(5 м + 5 %·s); выбор кадра выхода |

## 1. Одномерное состояние вдоль пути

**Hasberg, Hensel, Stiller, 2012.** C. Hasberg, S. Hensel, C. Stiller. Simultaneous localization and mapping for path-constrained motion. *IEEE Transactions on Intelligent Transportation Systems*, 13(2), 541–552, 2012. DOI [10.1109/TITS.2011.2177522](https://doi.org/10.1109/TITS.2011.2177522). [PDF](https://www.mrt.kit.edu/z/publ/download/2012/HasbergHenselStiller2012TrITS.pdf).
Взято: положение стеснённого рельсом экипажа — координата вдоль оси, курс даёт карта (`s` и касательная `assets/ring.csv`). В плане: онлайн-правка карты по повторным проездам (probabilistic curvemap).

**von Einem и др., ITSC 2023.** C. von Einem, A. Cramariuc, R. Siegwart, C. Cadena, F. Tschopp. Path-constrained state estimation for rail vehicles. *IEEE ITSC 2023*, 4600–4607. DOI [10.1109/ITSC57777.2023.10422075](https://doi.org/10.1109/ITSC57777.2023.10422075), [arXiv:2308.12082](https://doi.org/10.48550/arXiv.2308.12082).
Взята постановка: модель в 3 или 6 степенях свободы даёт положения вне рельса, известная карта восстанавливает позу из одной координаты. Их фильтр с visual-inertial одометрией и несколькими гипотезами на стрелках — ориентир для банка гипотез пути ([hypotheses.md](hypotheses.md)).

**Zhang и др., IEEE T-ITS 2025.** R. Zhang, C. Chen, H. Tang, Y. Qin, Z. Lai, Y. Bai. Onboard metro train localization based on the train motion and track geometry feature fusion. *IEEE Transactions on Intelligent Transportation Systems*, 26(10), 16275–16290, 2025. DOI [10.1109/TITS.2025.3571433](https://doi.org/10.1109/TITS.2025.3571433).
Один MEMS IMU и сопоставление движения с признаками геометрии пути; средняя ошибка ниже 1 % интервала. Подтверждает ставку на геометрию пути как источник абсолютной привязки. В плане: подписи геометрии как якоря между станциями (корреляция `log(v_front/v_rear)` с кривизной на маршруте 10 проверена и фазы не дала, подпись уклона остаётся).

**Zhang и др., ION GNSS+ 2026.** Y. Zhang, W. Jiang, J. Wang, B. Cai, J. Liu, X. Ba, D. Lu. 1D INS/ODO recursive navigation method for railway trains in GNSS-denied environments. *Proc. ION GNSS+ 2026*, Orlando, 14–18 Sep 2026, 631–645. [ION 20777](https://www.ion.org/publications/abstract.cfm?articleID=20777).
Одномерный рекурсивный фильтр INS/одометр: на 30 км Цинхай-Тибетской дороги RMSE пробега 1.335 м (против 16.073 м у 15-мерной слабосвязанной схемы) и скорости 0.0132 м/с. Это современная точка отсчёта одномерной постановки с IMU; RailBreak решает ту же задачу без IMU, заменяя второй канал моделью тяги и якорями станций, и выходит на сопоставимый метровый уровень на своём маршруте.

## 2. Робастное слияние при несогласии датчиков

**Palmer, Nourani-Vatani, IROS 2018.** A. W. Palmer, N. Nourani-Vatani. Robust odometry using sensor consensus analysis. *IEEE/RSJ IROS 2018*, 3167–3173. DOI [10.1109/IROS.2018.8594473](https://doi.org/10.1109/IROS.2018.8594473), [arXiv:1803.02237](https://arxiv.org/abs/1803.02237).
Взята идея: несогласному каналу скорости поднимают дисперсию, а не усредняют его остаток. В ноде это порог NIS и `R = 25 (м/с)²` у расходящейся тележки. Их вывод о том, что ускорение колеса при юзе лежит в пределах возможного разгона, объясняет, почему в ноде работает невязка к модели и согласие тележек, а не порог ускорения.

**Jang и др., 2026.** M. Jang, J. Lee, A. Hakobyan, N. Hovakimyan, I. Yang. Residual-aware distributionally robust EKF: absorbing linearization mismatch via Wasserstein ambiguity. [arXiv:2604.02749](https://arxiv.org/abs/2604.02749), 3 Apr 2026.
Объединяют несовпадение модели шума и ошибку линеаризации в множестве неопределённости Вассерштейна, получают вычислимый радиус робастности и детерминированные границы MSE; фильтр решает SDP на каждом шаге. Из этой линии в ноде — принцип «величина невязки раздувает неопределённость измерения» в виде O(1)-правила, совместимого с циклом 30 Гц. В плане: оценка радиуса неопределённости для таблицы тяги по невязкам `train`.

**González и др., 2026.** F. González, Ö. D. Akyildiz, D. Crisan, J. Míguez. An operator-theoretic analysis of nonlinear filtering under model misspecification. [arXiv:2607.11378](https://arxiv.org/abs/2607.11378), 2026.
Разделяют ошибку начального распределения и накопленную ошибку неверной динамики при верной модели наблюдения и дают условия, при которых фильтры с верной и неверной динамикой сходятся. Теоретическая рамка для разбора таблицы тяги как неверно заданной динамики; в ноду не переносится.

## 3. Сцепление и юз

**Pichlík, Bauer, IEEE TVT 2021.** P. Pichlík, J. Bauer. Adhesion characteristic slope estimation for wheel slip control purpose based on UKF. *IEEE Transactions on Vehicular Technology*, 70(5), 4303–4311, 2021. DOI [10.1109/TVT.2021.3072484](https://doi.org/10.1109/TVT.2021.3072484).
Наклон характеристики сцепления по возбуждению момента и угловой скорости колеса. Показывает, какие входы нужны для оценки сцепления; в контракте вагона их нет, поэтому публикуется карточка невязок ([adhesion.md](adhesion.md)).

**Bouchama и др., SysTol 2021.** H. F. Bouchama, D. Berdjag, M. Defoort, J. Lauber. Observer-based robust train speed estimation subject to wheel-rail adhesion faults. *5th Int. Conf. on Control and Fault-Tolerant Systems (SysTol)*, 303–310, 2021. IEEE [9594997](https://ieeexplore.ieee.org/document/9594997), [HAL hal-03406951](https://uphf.hal.science/hal-03406951).
Ближайшая постановка: робастная скорость при отказе сцепления; два наблюдателя — сила сцепления и скорость по бализам. Из неё в оценщике пять свойств: раздельные отказ измерения и отказ модели, конверт рядом с точкой, невязка колесо–модель, раздельное доверие каналов, время обнаружения и доля ложной тревоги на размеченной синтетике (`scenario_campaign.py`). Роль бализ у RailBreak играют якоря станций.

**Namoano и др., 2024.** B. Namoano, C. Emmanouilidis, A. Starr. Detecting wheel slip from railway operational data through a combined wavelet, long short-term memory and neural network classification method. *Engineering Applications of Artificial Intelligence*, 2024. DOI [10.1016/j.engappai.2024.109173](https://doi.org/10.1016/j.engappai.2024.109173).
Детектор юза по вейвлету и LSTM на эксплуатационных данных. В плане — теневой классификатор рядом с фильтром при наличии размеченных эпизодов юза и проверке на сдвиг домена.

**CMES 2025.** Online estimation method of train wheel-rail adhesion coefficient based on parameter estimation. *Computer Modeling in Engineering & Sciences*, 144(3), 2025. DOI [10.32604/cmes.2025.068951](https://doi.org/10.32604/cmes.2025.068951).
μ онлайн по силовой модели колёсной пары и динамике торможения.

**Zha и др., 2026.** W. Zha, D. Ou, Y. Tong, C. Xu, S. Su, J. Xiong. Train slide prediction and risk assessment using vehicle-signal data: a data-model fusion method. *Urban Rail Transit*, 12(3), 330–343, 2026. DOI [10.1007/s40864-026-00269-9](https://doi.org/10.1007/s40864-026-00269-9).
Двухканальная CNN–GRU с cross-attention по тормозному усилию, скорости поезда и скоростям колёсных пар прогнозирует вероятность юза на горизонте 2.5 с, модель тормозной динамики — отклонение от точки остановки. В плане — теневой предиктор юза, если тормозное усилие появится во входном контракте.

**Gan и др., ETAE 2026.** H. Gan, S. Wang, J. Lu, B. Xie. An LSTM-Kalman fusion-based hybrid observation method for wheel-rail adhesion coefficient estimation. *3rd Int. Conf. on Electrical Technology and Automation Engineering (ETAE)*, Shenzhen, 826–831, 2026. DOI [10.1109/etae69474.2026.11495772](https://doi.org/10.1109/etae69474.2026.11495772).
Наблюдатель μ по окружной скорости колеса и LSTM-приору.

## 4. Независимые каналы

**Potokar, McGann, Kaess, RA-L 2024.** E. Potokar, D. McGann, M. Kaess. Robust preintegrated wheel odometry for off-road autonomous ground vehicles. *IEEE Robotics and Automation Letters*, 9(12), 11649–11656, 2024. [PDF](https://www.cs.cmu.edu/~kaess/pub/Potokar24ral.pdf).
Факторный граф с предынтегрированием колёс, онлайн-оценкой радиуса и юза; требует IMU. Направление при появлении IMU во входном контракте.

**Dieckow и др., 2025.** N. Dieckow, K. Ostaszewski, P. Heinisch, H. Struckmann, H. Ranocha. Real-time rail vehicle localisation using spatially resolved magnetic field measurements. [arXiv:2507.19327](https://arxiv.org/abs/2507.19327), 2025.
Локализация по магнитной карте пути: фильтр частиц с модифицированным ядром — 2.07 м средней ошибки (максимум 5.3 м) на 21.6 км; холодный старт сопоставлением последовательностей. Кандидат на третий независимый канал, который делает общую моду двух тележек наблюдаемой (пункт 3 [плана](tz-audit.md#план-развития)).

## 5. Целостность и калиброванная неопределённость

**Maharmeh и др., Sensors 2025.** E. Maharmeh, Z. Alsayed, F. Nashashibi. A comprehensive survey on the integrity of localization systems. *Sensors*, 25(2), 358, 2025. [mdpi.com/1424-8220/25/2/358](https://www.mdpi.com/1424-8220/25/2/358).
Разделяет точность и заявление о целостности. RailBreak публикует оба: точность в метриках и эмпирическую границу с флагом `certification_claim = false` ([integrity.md](integrity.md)).

**Kumar и др., 2025.** D. Kumar, S. Tayebati, F. Migliarba, R. Krishnan, A. R. Trivedi. Learnable conformal prediction with context-aware nonconformity functions for robotic planning and perception. [arXiv:2509.21955](https://arxiv.org/abs/2509.21955), 2025.

**Hore и др., 2026.** R. Hore, A. Chatterjee, S. Choudhury. Multi-source conformal prediction: leveraging heterogeneity via localization. [arXiv:2609.14531](https://arxiv.org/abs/2609.14531), 13 Sep 2026.
Конечновыборочное покрытие при объединении разнородных источников. Прямо ложится на два вагона с разным масштабом (30618 и 30639).

**Lai, Raskutti, 2026.** Y. Lai, G. Raskutti. Online localized conformal prediction. [arXiv:2605.05497](https://arxiv.org/abs/2605.05497), 2026.
Локализованная по признакам калибровка с онлайн-обновлением уровня промаха; долгосрочное покрытие приближается к целевому при дрейфе. Путь от статического квантиля `train` к границе, которая держит покрытие на новых датах.

Текущая граница `B_s` — квантиль `train`, проверенный на `val`. Для конформной гарантии нужны отдельный калибровочный сплит, непройденные даты и вагоны и учёт зависимости соседних фиксов одного рейса.

## 6. Нормативные линейки

**UNISIG SUBSET-041**, Performance Requirements for Interoperability, v3.2.0, §5.3.1.1. [ERA PDF](https://www.era.europa.eu/system/files/2023-01/sos3_index014_-_subset-041_v320.pdf).
Используется как внешняя линейка величин: точность одометрии ±(5 м + 5 %·s) от репера и свежесть оценки моложе 1 с. По этой линейке медиана `val` (1.47 м вдоль пути, дрейф 0.024 % пути) и скорость (0.032 м/с при допуске ±0.56 м/с до 30 км/ч) укладываются с запасом; RailBreak — трамвайная система вне ETCS, соответствие SUBSET-041 не заявляется.

**ГКИНП (ОНТА)-01-268-02.** Основные положения по созданию и обновлению опорной геодезической сети г. Москвы; приказ Москомархитектуры от 20.01.2003 № 13, «Вестник Мэра и Правительства Москвы» № 18, март 2004. [ГАРАНТ](https://base.garant.ru/4093150/).
На разборе этим документом называли МКРС (начало 55°40′ с.ш., 37°30′ в.д., эллипсоид Бесселя). Эксперты затем подтвердили MGRS; по умолчанию выход — UTM 37N минус угол квадрата 300 000 / 6 100 000 м, МКРС доступна параметром `output_frame`.

## 7. Решения трека на записи `30618_88aea4d9`

Числа — из README и отчётов авторов; в нашем Docker их узлы не перезапускались.

| Решение | Подход | Скорость, м/с | 3D, м |
|---|---|---:|---:|
| **RailBreak** | Шмидт — Калман `[s, v, k, bₐ]`, геодезическое кольцо, среднее тележек, `notch_fault`, монитор целостности | **0.024691** | **1.157702** (макс. 3.67) |
| [`67-time-to-win-hack`](https://github.com/Kvascs/67-time-to-win-hack) (`tbo_core`) | GPB1 на пяти режимах, CUSUM, подпись `log(v_front/v_rear)`, масштаб по шагу квантования, западный тупик в карте | 0.026 | 0.60 |
| `ekimlichenko-mt` | Одномерный оцениватель с GNSS по ходу рейса (без него 7.16 м) | 0.0324 | 1.111 (макс. 14.02) |
| [`VECTRA`](https://github.com/kitay-sudo/vectra-tram-odometry) | UKF `[s, v, d, k_т, k_б]`, сопротивление Дэвиса, сдвиг скорости 0.08 с (без него 0.059 м/с) | 0.042 | 1.43 |
| [`tram-backup-odometry`](https://github.com/KroLor/tram-backup-odometry) | Эхо-сеть и Калман `[v, b]`, профиль ветки | 0.051 | 1.209 |
| `dub-otrezkov` | Проекция на путь | 0.055 | 1.053 (макс. 13.17) |
| [`FunnyRepo`](https://github.com/MikeEgorshev/FunnyRepo) (`2afdb9f`) | Таблица 31×16, задержка тяги 0.4 с | 0.033 | 5.13 |

Ни один чужой узел, модель или параметр в ноду не переносились. Общий физический признак с `tbo_core` — проверка знака ручки против согласованного ускорения тележек; в RailBreak это собственное правило `notch_fault` без их CUSUM и GPB1. Сетка сохранённой дуги 0.04 с — собственное правило ноды.

## 8. Соседние репозитории KonkovDV

Из [wellguard-os](https://github.com/KonkovDV/wellguard-os), [open-integrity-twin](https://github.com/KonkovDV/open-integrity-twin) и [drillguard-os](https://github.com/KonkovDV/drillguard-os) взято правило объяснимой карточки: плохой вход, невязка одного канала и общая невязка — разные детерминированные метки, которые не пишут фильтр. В диагностике это `advisory_class`.

## 9. Внешние справки

- [mos.ru, тема 13333050](https://www.mos.ru/mayor/themes/13333050/) — основная локализация беспилотного трамвая сравнивает сцену с лидарной картой; RailBreak — резервный канал на штатных датчиках вагона.
- [DEUTA: odometry and speed-distance unit](https://deuta.com/en/odometry-and-speed-distance-unit.aspx) — однотипные датчики ошибаются вместе; отсюда разделение каналов «колёса» и «модель тяги».
