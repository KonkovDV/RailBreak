<p align="center"><img src="../../Logo.png" alt="RailBreak" width="280"></p>

# Источники сдаваемого фильтра

Полные записи к упоминаниям в `model.md`, `assumptions.md`, `form.md` и `pitch.md`.
Ссылка не переносит в ноду метод статьи. Ниже отдельно: что взято и что сделано в этом репозитории.

Кольцо, остановки, таблица ручки, состояние [s, v, k, bₐ], якорь по ZUPT, перевод км/ч, выход `base_link` и скорер написаны здесь. Их нет в перечисленных статьях.

## Hasberg, Hensel, Stiller, 2012

C. Hasberg, S. Hensel, C. Stiller. Simultaneous localization and mapping for path-constrained motion. *IEEE Transactions on Intelligent Transportation Systems*, vol. 13, no. 2, pp. 541–552, 2012. DOI [10.1109/TITS.2011.2177522](https://doi.org/10.1109/TITS.2011.2177522). ISSN 1524-9050. Открытый PDF института: [HasbergHenselStiller2012TrITS.pdf](https://www.mrt.kit.edu/z/publ/download/2012/HasbergHenselStiller2012TrITS.pdf).

Взято одно: положение стеснённого рельсом экипажа — координата вдоль оси, курс даёт карта. В ноде это s и касательная `assets/ring.csv`.

Не взято: probabilistic curvemap, кубические сплайны, одновременное обновление карты и фильтра, их EKF. Пункт плана про онлайн-правку карты по повторным проездам на эту статью только указывает. В текущем фильтре карта заморожена после train.

## von Einem, Cramariuc, Siegwart, Cadena, Tschopp, ITSC 2023

C. von Einem, A. Cramariuc, R. Siegwart, C. Cadena, F. Tschopp. Path-constrained state estimation for rail vehicles. *2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)*, Bilbao, 24–28 Sep. 2023, pp. 4600–4607. DOI [10.1109/ITSC57777.2023.10422075](https://doi.org/10.1109/ITSC57777.2023.10422075). Препринт: [10.48550/arXiv.2308.12082](https://doi.org/10.48550/arXiv.2308.12082).

Взята постановка. Состояние рельсового экипажа — одномерная координата вдоль известной геометрии пути. Статья прямо пишет, что модель в 3 или 6 степенях свободы даёт положения вне рельса, а известная карта восстанавливает позу из этой координаты. В ноде это s, проекция p(s) и касательная.

Не взято: их EKF, перевод чужих измерений в одномерное пространство, несколько гипотез на стрелках, visual-inertial одометрия и постоянный GNSS. В ноде одно кольцо, после старта нет ни зрения, ни IMU, ни выбора ветки. Отдельный офлайн-прототип в [hypotheses.md](hypotheses.md) считает вес wⱼ на синтетической стрелке и в ноду не входит. RMSE 4.78 м и track selectivity 94.9 % — числа той статьи, не RailBreak.

## Pichlík, Bauer, IEEE TVT 2021

P. Pichlík, J. Bauer. Adhesion characteristic slope estimation for wheel slip control purpose based on UKF. *IEEE Transactions on Vehicular Technology*, vol. 70, no. 5, pp. 4303–4311, 2021. DOI [10.1109/TVT.2021.3072484](https://doi.org/10.1109/TVT.2021.3072484).

Их метод считает наклон характеристики сцепления по возбуждению момента и угловой скорости колеса. В ноде этих входов нет, оценщик не перенесён.

Не взято: их UKF, определение сдвига фазы между моментом и скоростью колеса, контур противоюза. Публикуется прокси невязок в [adhesion.md](adhesion.md): согласие тележек, невязка к модели, NIS, длительность общей моды, вырезка и скорость. `mu_estimate` всегда null.

## Palmer, Nourani-Vatani, IROS 2018

A. W. Palmer, N. Nourani-Vatani. Robust odometry using sensor consensus analysis. *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Madrid, 1–5 Oct. 2018, pp. 3167–3173. DOI [10.1109/IROS.2018.8594473](https://doi.org/10.1109/IROS.2018.8594473). Препринт [arXiv:1803.02237](https://arxiv.org/abs/1803.02237).

Взята идея: несогласному каналу скорости не усредняют остаток, а поднимают дисперсию. В ноде это порог NIS и R = 25 (м/с)² у расходящейся тележки.

Не взято: их процедура sensor consensus analysis и оценка диаметра колеса как состояния EKF по независимому датчику скорости. Здесь k — consider-состояние, на колёсах Kₖ = 0, двигают его только якоря. Ускорение колеса при юзе может оставаться в диапазоне физически возможного разгона. Одного порога ускорения поэтому мало: в ноде остаются несогласие с моделью и общая мода.

## Направления рядом, в ноду не взяты

Это не методы сдачи. Их числа в репозиторий не переносятся.

H. F. Bouchama, D. Berdjag, M. Defoort, J. Lauber. Observer-based Robust Train Speed Estimation Subject to Wheel-Rail Adhesion Faults. *5th International Conference on Control and Fault-Tolerant Systems (SysTol)*, Saint-Raphaël, 29 Sep.–1 Oct. 2021, pp. 303–310. IEEE [9594997](https://ieeexplore.ieee.org/document/9594997), запись [HAL hal-03406951](https://uphf.hal.science/hal-03406951). Близкая постановка: робастная продольная скорость при отказе сцепления, который портит измерение скорости. В статье два наблюдателя: скользящий дифференцирующий фильтр силы сцепления за конечное время и непрерывно-дискретный high-gain наблюдатель скорости по непериодическим бализам. Оба в ноду не входят. Силы сцепления и бализ здесь нет. Полная замена фильтра их схемой не делается. Числа их симуляций не являются числами RailBreak.

Из этой постановки в текущем оценщике уже есть пять свойств, без их наблюдателей.

Отказ измерения и отказ модели разделены. Одна тележка в стороне от другой — канал измерения: растёт R, доверие этой тележки падает. Обе согласны и обе в стороне от модели — не отказ одной тележки. Причина не называется: это может быть модель, задержка, масса, уклон или общий юз. После `recover_s` = 3 с политика `COMMON_MODE_UNOBSERVABLE`, затем слепой бюджет и `LOST`.

Ограничение публикуется рядом с точкой, а не как доказанная практическая устойчивость их наблюдателя. Конверт — `empirical bound, not certified protection level`. В `pose.covariance` он не вкладывается.

Невязка отказа — колесо минус модель и NIS. Это не невязка их дифференцирующего фильтра и не юз. `SLIP_SUSPECTED` не публикуется.

Доверие раздельное: скорость, положение, каждая тележка и модель, со значениями `HIGH`, `LOW`, `NONE`.

Задержка обнаружения и ложная тревога считаются на размеченном синтетическом прогоне в `scenario_campaign.py`: `time_to_detection`, `false_alarm_rate`, `missed_detection_rate`. Пустое множество остаётся пустым, а не нулём. Эти доли не заменяют 1.467 м и не являются evidence текущего HEAD.

B. Namoano, C. Emmanouilidis, A. Starr. Detecting wheel slip from railway operational data through a combined wavelet, long short-term memory and neural network classification method. *Engineering Applications of Artificial Intelligence*, 2024. DOI [10.1016/j.engappai.2024.109173](https://doi.org/10.1016/j.engappai.2024.109173). Детектор юза по вейвлету, LSTM и нейросетевой классификации. В сегодняшнюю сдачу не входит и шаг фильтра не заменяет. Их метрики не являются метриками RailBreak.

После дедлайна это может быть только теневой классификатор. Пять причин, почему не сейчас. Нужны лейблы юза, а в этом дереве подтверждённый юз не публикуется. Нужна репрезентативная выборка, не один знакомый синтетический профиль. Высок риск domain shift между генератором, train и скрытым test. Генератор нельзя подгонять под классификатор: семейства A–F и `truth_sim.py` тогда перестают быть незнакомой истиной. Не видно, как выход классификатора гарантирует поведение целостности: `LOST`, слепой бюджет и конверт остаются политикой модели, а не решением сети.

E. Potokar, D. McGann, M. Kaess. Robust Preintegrated Wheel Odometry for Off-road Autonomous Ground Vehicles. *IEEE Robotics and Automation Letters*, vol. 9, no. 12, pp. 11649–11656, Dec. 2024. PDF [CMU](https://www.cs.cmu.edu/~kaess/pub/Potokar24ral.pdf). Сильное направление для колёсной одометрии, но это roadmap, а не правка к дедлайну. Их факторный граф требует IMU, трёхмерное движение, онлайн-оценку радиуса и юза и оптимизацию. В этой ноде IMU нет, движение — дуга кольца, k на колёсном шаге не двигается, факторного графа нет. Их числа не являются числами RailBreak.

Online Estimation Method of Train Wheel-Rail Adhesion Coefficient Based on Parameter Estimation. *CMES*, vol. 144, no. 3, 2025. DOI [10.32604/cmes.2025.068951](https://doi.org/10.32604/cmes.2025.068951). Там μ считается онлайн-оценщиком параметра по силовой модели одной колёсной пары и динамике торможения. В ноду этот оценщик не входит. Их симуляция не является числом RailBreak.

Для RailBreak `mu_estimate = null` — правильное решение. Настоящая оценка сцепления требует момент тяги или торможения, модель силы, динамику поезда, задержку привода, вращательную динамику колеса и независимый репер. Две скорости тележек и команда водителя этого набора не дают: коэффициент сцепления неидентифицируем.

> coefficient of adhesion is not observable from two wheel speeds and driver command alone.

E. Maharmeh, Z. Alsayed, F. Nashashibi. A Comprehensive Survey on the Integrity of Localization Systems. *Sensors*, 2025, 25(2), 358. [mdpi.com/1424-8220/25/2/358](https://www.mdpi.com/1424-8220/25/2/358). Обзор разделяет точность и заявление о целостности или protection level. Граница этой сдачи остаётся `empirical bound, not certified protection level`.

F. González, Ö. D. Akyildiz, D. Crisan, J. Míguez. An Operator-Theoretic Analysis of Nonlinear Filtering under Model Misspecification. arXiv:[2607.11378](https://arxiv.org/abs/2607.11378). Разбор фильтра при неверной динамике. В ноду не перенесён. Это материал для разбора устойчивости после дедлайна, не для текущего шага.

M. Jang, J. Lee, A. Hakobyan, N. Hovakimyan, I. Yang. Residual-Aware Distributionally Robust EKF: Absorbing Linearization Mismatch via Wasserstein Ambiguity. arXiv:[2604.02749](https://arxiv.org/abs/2604.02749), 3 Apr 2026. Наиболее интересное теоретическое направление для следующей версии: несовпадение модели шума, несовпадение линеаризации, неоднозначность Вассерштейна и детерминированные границы MSE. Их фильтр на каждом шаге решает SDP и в ноду не входит. Их симуляции не являются числами RailBreak.

Для сегодняшней версии взята только идея: величина невязки раздувает неопределённость измерения. В коде это порог ν²/S > 16, после которого R = 25 (м/с)². Это не радиус Вассерштейна и не детерминированная граница MSE. Конверт остаётся `empirical bound, not certified protection level`.

D. Kumar, S. Tayebati, F. Migliarba, R. Krishnan, A. R. Trivedi. Learnable Conformal Prediction with Context-Aware Nonconformity Functions for Robotic Planning and Perception. arXiv:[2509.21955](https://arxiv.org/abs/2509.21955). Страница: [divake.github.io/learnable-cp-robotics](https://divake.github.io/learnable-cp-robotics/). R. Hore, A. Chatterjee, S. Choudhury. Multi-source conformal prediction: leveraging heterogeneity via localization. arXiv:[2609.14531](https://arxiv.org/abs/2609.14531), 13 Sep 2026. Хорошее направление для эмпирического интервала. В эту сдачу их процедуры не входят, и их покрытия не являются числами RailBreak.

Текущая граница Bₛ — квантиль train, проверенный на val. Это не conformal prediction. Чтобы интервал стал конформным, нужны честный calibration split, не те же рейсы, на которых выбран квантиль; непройденные маршруты и вагоны, а не только маршрут 10; и учёт зависимости временного ряда: соседние фиксы одного рейса не обменны. Даже тогда это не SIL proof и не `certified protection level`. `certification_claim` остаётся `false`.

Общий вывод. К дедлайну сильнее не нейросеть, а интерпретируемый оцениватель по модели, явные гипотезы отказа, консервативная машина состояний целостности, явное объявление ненаблюдаемости и воспроизводимый прогон.

## ГКИНП (ОНТА)-01-268-02

Основные положения по созданию и обновлению опорной геодезической сети г. Москвы. Введены приказом Москомархитектуры от 20 января 2003 г. № 13. Опубликованы в «Вестнике Мэра и Правительства Москвы», № 18, март 2004, приложение 2 к постановлению Правительства Москвы от 2 марта 2004 г. № 115-ПП. DOI нет. Публично читаемая копия этой публикации: [ГАРАНТ](https://base.garant.ru/4093150/).

На разборе этим документом называли МКРС. Раздел 4 и таблица 1 задают частную систему: начало 55°40′ с.ш., 37°30′ в.д. В ноду это не перенесено. Выход по умолчанию — UTM зоны 37N минус угол квадрата 300000 м / 6100000 м. Эллипсоид Бесселя и переход WGS84→Бессель не применяются.

## UNISIG SUBSET-041

UNISIG SUBSET-041, Performance Requirements for Interoperability, v3.2.0. Официальный файл ERA: [subset-041_v320.pdf](https://www.era.europa.eu/system/files/2023-01/sos3_index014_-_subset-041_v320.pdf), §5.3.1.1.

Взято как внешняя линейка величин: вид ±(5 м + 5 %·s) и свежесть оценки моложе 1 с. Реализация требований SUBSET-041, ETCS и сертификат не заявляются. Штамп выхода равен штампу входа; это не заявка на соответствие пункту.

## Сообщение мэрии

[mos.ru, тема 13333050](https://www.mos.ru/mayor/themes/13333050/). Внешняя справка о том, что основная локализация сравнивает сцену с лидарной картой и не зависит от ГЛОНАСС. В подписки ноды лидар не входит.
