# Графики питча

Синтетика, seed 42, заявленная версия ядра `tramDR-0.0.11`.
Чёрный — GT, синий — UKF, пунктир — конверт чекера (5 м + 5% пути).
Не KPI маршрута 10 и не bag организатора.

На baseline `38f65c1` каталог содержит **23 SVG**, а не 18:

- **19 основных сценариев:** `all_encoders_dead`, `axle_fault`, `coast_no_wire`,
  `diameter_wear`, `grade_unmapped`, `heavy_pax`, `jagged_notch`, `mismatch_jerk`,
  `mismatch_jerk_slip`, `mismatch_r0`, `model_mismatch`, `six_axle`,
  `slide_brake`, `slide_on_grade`, `slip_accel`, `snow_ice`, `tight_curve`,
  `wet_clean`, `zupt_dwell`.
- **4 сценария оценки моста, также присутствуют в каталоге:** `coast_grade_route10`,
  `slide_on_grade_route10`, `slide_on_grade_route10_steep`, `grade_traction_route10`.
  Не $i(s)$ в ноде. 32 ‰ — сценарная оценка, не установленный факт.

Состав сверен по именам файлов; это не визуальная проверка SVG/PPTX
и не повторное вычисление траекторий. Наличие графиков не заменяет raw logs,
commit/параметры и хеши входов. Таблицы и ограничения —
[`docs/metrics.md`](../../docs/metrics.md).

```bash
python tools/eval/plot_run.py --runs synth/runs --out-dir evidence/plots-pitch
```
