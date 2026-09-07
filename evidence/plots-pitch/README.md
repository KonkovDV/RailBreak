# Графики питча

Синтетика, seed 42, ядро `tramDR-0.0.9`.
Чёрный — GT, синий — UKF, пунктир — конверт чекера (5 м + 5% пути).
Не KPI маршрута 10 и не bag организатора.

18 сценариев близнеца: `axle_fault`, `coast_no_wire`, `diameter_wear`, `grade_unmapped`,
`heavy_pax`, `jagged_notch`, `mismatch_jerk`, `mismatch_jerk_slip`, `mismatch_r0`,
`model_mismatch`, `six_axle`, `slide_brake`, `slide_on_grade`, `slip_accel`,
`snow_ice`, `tight_curve`, `wet_clean`, `zupt_dwell`.

Оценка моста (не в этом пакете графиков): `coast_grade_route10`,
`slide_on_grade_route10`, `slide_on_grade_route10_steep`,
`grade_traction_route10`. Не $i(s)$ в ноде. 32 ‰ — не факт.

```
python tools/eval/plot_run.py --runs synth/runs --out-dir evidence/plots-pitch
```
