# Эмпирическая граница ошибки

Теневой монитор читает уже посчитанные признаки резервной одометрии и публикует статус, причины и границу вдоль пути. Состояние фильтра он не пишет.

Название числа: **empirical along-track integrity bound**, экспериментальная граница ошибки. Это не сертифицированный protection level. `certification_claim` всегда `false`.

Координату можно использовать, пока статус не `POSITION_UNTRUSTED`. Такой статус ставится, если штамп шагнул назад, если обе тележки старше 30 с, или если нет абсолютного старта и точка старта вне карты.

## Граница

\[
q_{0.99}\,\sigma_s + B_{\text{mode}} + B_{\text{time}} + B_{\text{map}}
\]

`q_0.99 = 8.264474` — объединённый 99-й перцентиль `|e_s| / sigma_s` на чистом train. Брались фиксы со статусом `NOMINAL` и `sigma_s` не меньше 0.05 м: 400344 фикса, 39 записей из 61. В выборку вошла запись, у которой после окна GNSS на кольце осталось не меньше 100 фиксов. Validation в подгонку не входил. Скрытый test не читался.

| статус | `B_mode`, м | фиксов в подгонке |
| --- | ---: | ---: |
| `NOMINAL` | 0 | 400344 |
| `DEGRADED_SINGLE_BOGIE` | 10.019 | 22333 |
| `DEGRADED_MODEL_CARRY` | 0 | 10, порог 30 не набран |
| `DEGRADED_COMMON_MODE_UNOBSERVABLE` | 32.606 | 149 |
| `DEGRADED_NO_MAP` | 0 | 0, на train такого старта не было |
| `DEGRADED_RELATIVE_ONLY` | 0 | 0, на train такого старта не было |
| `POSITION_UNTRUSTED` | 0 | 0, на train такого старта не было |

`B_mode` для испорченного статуса — 99-й перцентиль `max(0, |e_s| - q·sigma_s)` на чистом train и на том же train с масштабом задней тележки +5 % на весь рейс. Фиксы этого масштаба, которые монитор оставил в `NOMINAL`, в номинальный запас не входят.

`B_time = 0`. На train с одновременным пропаданием обеих тележек на 5 с нашлось 1723 фикса, у которых более свежая тележка старше 0.35 с. 99-й перцентиль остатка сверх `q·sigma_s + B_mode` равен нулю.

`B_map = 0`. Эталон — само кольцо, отдельный вклад карты не выделен.

На чистом val (21 запись из 22) внутри границы лежит 0.972 фиксов. Медиана по записям — 1. Покрытие 0.99 относится к номинальному train, на котором взят перцентиль.

Масштаб задней тележки +5 % на весь train монитор в основном не переводит из `NOMINAL`: 378410 таких фиксов остались номинальными, 99-й перцентиль `|e_s|` по ним 230.6 м. Граница этого случая не покрывает. Пока статус номинальный, число вдоль пути не говорит, что постоянный масштаб обеих согласованных тележек пойман.

Пример полей диагностики, коэффициенты из train:

```yaml
integrity:
  status: DEGRADED_COMMON_MODE_UNOBSERVABLE
  along_bound_m: <8.264474 * sigma_s + 32.606>
  calibrated_coverage: 0.99
  calibration_split: train
  certification_claim: false
```

Число 14.2 из постановки сюда не подставлялось.

## Статусы

`NOMINAL`, `DEGRADED_SINGLE_BOGIE`, `DEGRADED_MODEL_CARRY`, `DEGRADED_COMMON_MODE_UNOBSERVABLE`, `DEGRADED_NO_MAP`, `DEGRADED_RELATIVE_ONLY`, `POSITION_UNTRUSTED`.

Причины: `FRONT_NIS_HIGH`, `REAR_NIS_HIGH`, `BOGIES_DISAGREE`, `BOGIES_AGREE_MODEL_DISAGREES`, `STALE_FRONT`, `STALE_REAR`, `STAMP_REGRESSION`, `NO_ABSOLUTE_START`, `AMBIGUOUS_STATION_ANCHOR`, `MAP_OUT_OF_DOMAIN`.

`BOGIES_AGREE_MODEL_DISAGREES` держится после эпизода, в котором обе тележки согласны между собой и обе долго расходятся с моделью. Сбрасывается, когда тележки расходятся друг с другом или когда счётчик якорных остановок вырос. `AMBIGUOUS_STATION_ANCHOR` — причина на стоянке, если в воротах больше одной остановки. Фильтр по-прежнему не выбирает из них.

Правила совпадают в `integrity_monitor.hpp` и `tools/organizer/integrity.py`. Коэффициенты лежат в `integrity_bound.hpp` и `assets/integrity_bound.json`. Узел добавляет поля `integrity_*` в `/result/diagnostics`.
