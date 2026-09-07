# Литература

Только то, что реально стоит за кодом или за формулировкой доклада.
Не SR-UKF, не CKF как продукт (флаг `ukf_cubature` — опция на близнеце).
Не Полах в ноде.

Канон наблюдаемости и целостности: [`brief.md`](brief.md).

| | Где | Зачем |
| --- | --- | --- |
| Julier 2002 (scaled UT); Luo & Moroz 2009 (PSD) | `ukf.cpp` $\alpha=0.58$ | необходимое $W_c^{(0)}\ge 0$ даёт окно $\alpha\in[0.517638,\,1.931852]$ при $\beta=2$ и **не зависит от $L$** ($\beta=1$: $[0.618034,\,1.618034]$; $\beta=0$: ровно $\{1\}$); достаточное $1/\sqrt{1+\beta}=0.577350$, выбранное 0.58 — запас $+0.459\,\%$ |
| Arasaratnam & Haykin, IEEE TSP 2009 | `UkfParams.cubature` | 2L точек; по умолчанию выкл. |
| Higham (PD) | `lin_alg.hpp` `project_pd` | Якоби + клип $\lambda$; это не square-root UKF. Почему downdate здесь не нужен — [`estimator-priors.md`](estimator-priors.md) §3. Измеренное возмущение до правки: $\max\lvert\Delta P\rvert=0.109701$ на плотной SPD $12\times12$, $2.39\cdot10^{-7}$ на реалистичной $P$ ($\mathrm{cond}\approx 2\cdot10^{10}$); после правки — no-op на PD-входе |
| Kulikova & Kulikov, IFAC 2020; обзор arXiv:2406.05188 | отсутствие SR-фильтра | «previously suggested Cholesky-based UKF implementations are, in fact, the *pseudo* square-root versions… the resulting downdated matrix might be not a positive definite matrix». Возражение про **downdate**; при всех $W_c>0$ downdate не возникает, поэтому обычный UKF защитим. https://ifatwww.et.uni-magdeburg.de/ifac2020/media/pdfs/0536.pdf |
| Bar-Shalom, Li, Kirubarajan 2001 | `score.py` NEES | полосы консистентности; информативны на mismatch, не на twin |
| Or, arXiv:2512.18508 (2025) | докладываемый `nis` | после гейта/капа невязка распределена как **усечённый** $\chi^2$, а не $\chi^2_m$ — общая форма находки F-24. https://arxiv.org/pdf/2512.18508 |
| Willsky & Jones 1976; Isermann 2006 | детектор $\kappa$ | рамка parity / GLR; в коде — hold + знак остатка, не полный GLR |
| Palmer & Nourani-Vatani, IROS 2018 | SCA inflate | inflate $R$, не hard-delete |
| Malvezzi, Allotta, Rinchi, VSD 2011; Allotta et al. 2002 | питч | второй принцип в ATP-одометрии; калибровка на выбеге |
| Lauer & Stein, IEEE T-ITS 2015 | питч | локализация на 1D-графе пути |
| Hasberg, Hensel, Stiller 2012 | питч | path-constrained; 1D-многообразие |
| Brocard et al. 2020 | после bag | карта как априори, не сенсор |
| Kim et al. 2015 | питч (контраст) | slip/slide + adaptive sharing **с IMU**; tramDR — без IMU, потолок модели |
| UNISIG SUBSET-041 v3.2.0 | `al_s_m` | $\mathrm{AL}_s = 5 + 0.05\,s$ — **ровно** конверт одометрии ETCS $\pm(5\,\text{м} + 5\,\%\cdot s)$; там же «shall evaluate a safe confidence interval» при неисправности. Допуск взят из документа, а не назначен. https://www.era.europa.eu/system/files/2023-01/sos3_index014_-_subset-041_v320.pdf |
| ION, «Autonomous Integrity Monitoring Proposal for Critical Rail Applications» | `k_over`, `k_sigma` | THR SIL-4 $\le 2\cdot10^{-9}$/ч на систему сигнализации, доля на GNSS до $10^{-11}$/ч. Отсюда $k$ — **аллокация риска**, а не настройка: [`integrity-risk.md`](integrity-risk.md). https://www.ion.org/publications/abstract.cfm?articleID=12944 |
| Combino NF100, ITSC 2020 | `vehicle_combino_nf100.yaml` | twin: тара 28 т, $a_{\mathrm{trac}}=1.3$; не московский вагон |
| 71-911ЕМ Львёнок-Москва | `vehicle_lvenok_moscow.yaml` | Bo-Bo, все motor. $m_0$/Ø не в YAML: Википедия 22 т (семейство); каталог-копия ≤24 т / Ø 620 мм; TransPhoto ЕМ-03 25.2 т, 4×72 кВт. Клип [15, 40] т |
| ПК ТС, лист «Львёнок» | https://pk-ts.org/produkciya/l-venok/ | офиц. таблица: 16700×2500 мм, 40 мест, 101–161 чел.; **тары, кВт, Ø нет** |
| TransPhoto 71-911ЕМ-03 | https://transphoto.org/page/671/ | «Львёнок-Москва» 2021, 40 ед.: тара 25.2 т, 4×72 кВт, НЭ-001, автоход 1000 м — не беспилот ЦБТ |
| mos.ru, 02.10.2024 | питч $v_{\max}$ | беспилот 60 км/ч; датчики ставят после завода. Не путать с автономным ходом |
| Собянин, 08.01.2026 | словарь | маршрут 10 = беспилот ЦБТ; пр. Сахарова / №90 = автономный ход без КС; 50 автономных вагонов 2025 ≠ беспилотники |
| ПГУПС / РЭ 71-931М | `vehicle_vityaz_m.yaml` | тара 37 т; $a_{\max}$/Ø не утверждаются; не default маршрута 10 |
| EN 15595:2018 (+A1:2023); UIC 541-05 (2016) | словарь, генератор | relative wheel slide, WRM; lock $\le 0.4\,\mathrm{s}$ — не сертификация |
| EN 13452; ГОСТ 8802 / ПТЭ трамвая | питч | городской рельс: контекст, не «соответствует» |
| RAIB Report 08/2026 (18.06.2026); IR1/2025 (апрель 2025) | `slide_on_grade` | юз на уклоне 1:56 — рис. 46 финала; WSP ~1 с после ступени 3; EB = та же сила, другой контур (п. 86); 9 рекомендаций; 4 дефекта песочниц — IR1. Не путать MH1080 с «1080 м» |
| Zhu et al., Wear 2019 | классы $\mu$ | contaminated film $\le 0.05$, не «дождь» |
| Kuhse et al. 2023; Bédard et al. (ros2_tracing) | нода | depth-1, событийный предикт; p99 live |
| Masreliez 1975; Agamennoni et al. 2012 | Huber inflate | M-оценивание, cap информации |
| Davis ($A+Bv+Cv^2$) | `davis_resistance_n` | символы $A_d,B_d,C_d$, не Полах $A,B$ |
| Wear / Polach 2005 (9)(11) | только `generator_step` | генератор; в UKF нет |
| WILAC / contaminated film | синтетика `mu=0.06` | не называть дождём |
| Zhang et al. CMES 2025; Shrestha PhD 2025 | $\hat\mu$ в $x$ | оценка адгезии требует момента — клип, не observer |
| Reid et al. 2019; IM review 2025 | чекер | PL / AL / HMI как метрика |
| OSM ODbL | `route_10.yaml` | вершины остановок OSM (9 точек), не ось пути и не $i(s)$; 5.5 км не цитировать; ginfo: 9 туда / 8 обратно (без Бурназяна) |
| Википедия «Строгинский мост»; Носарев–Скрябина, «Мосты Москвы», 2004, с. 224–225 | генератор `*route10*` | рамно-подвесной, 5 пролётов + эстакада, 570 м. Не круизные «арочный 120 м». Уклон: центр 32 ‰, 25–40 ‰ — оценка, не в ноду |
| mos.ru / АГН Москва / Газета Метро, 13–17.06.2026 | inspect_bag, bag 25.09 | капремонт русловой части, временная линия, расписание штатное. Режим июня 2026, не майский заголовок. Срок не объявлен |

`stop_associate.py` читает полилинию остановок и JSONL оценки. Это не
измерение UKF и не map-matching в $z$.

## Внутренние артефакты проверки

Не литература, а то, на что опираются числа в таблице выше и утверждения доклада.
Разделёно специально: ссылка на свой же тест — не ссылка на внешний источник.

| Артефакт | Зачем |
| --- | --- |
| [`../standalone/test_ut_weights_psd.cpp`](../standalone/test_ut_weights_psd.cpp) | 25 проверок алгебры весов scaled UT (строка Julier / Luo–Moroz); ядро не линкуется, поэтому тест независим от фильтра |
| [`../standalone/test_integrity_contracts.cpp`](../standalone/test_integrity_contracts.cpp) | 15 контрактов целостности: свидетельство стоянки, свежесть во времени, контракт входа, атомарный откат |
| [`../standalone/test_prior_and_nis.cpp`](../standalone/test_prior_and_nis.cpp) | F-23 и F-24 численно: неподвижная точка дефектной рекуррента (закрытая форма **и** итерация), точное тождество сжатия $P_{k+1}-R=\varphi^2(P_k-R)$, предел разрешения реверсии среднего, насыщение NIS на $c^2$ |
| [`integrity-risk.md`](integrity-risk.md) | бюджет целостности: $k \leftrightarrow$ риск, происхождение AL, связующий перегон маршрута 10 |
| [`estimator-priors.md`](estimator-priors.md) | алгебра F-23/F-24 и доказательство, почему обычный UKF здесь допустим |
| [`verification.md`](verification.md) | что исполнялось численно, что выведено, что осталось руками |
| [`../CHANGELOG.md`](../CHANGELOG.md) | реестр находок `F-01`…`F-17` и их статус |
| [`review/external-triage-2026-09-06.md`](review/external-triage-2026-09-06.md) | внешний триаж: исходные формулировки находок |
| [`audit-2026-09-06.md`](audit-2026-09-06.md) | самозаявленные ограничения ветки правок (что не запускалось) |
| [issue #3](https://github.com/KonkovDV/RailBreak/issues/3) | датированный снимок открытых пунктов |
