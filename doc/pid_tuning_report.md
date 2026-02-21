# PID Tuning Report Template

## Metadata
- Timestamp: {{timestamp}}
- Robot: {{robot}}
- Profile: {{profile}}
- Budget (min): {{budget_min}}
- Sampling Rate (Hz): {{sample_hz}}

## Environment
- Launch: `roslaunch rm_description sentry.launch paused:=true roller_type:=simple`
- Controller: `sentry_chassis_controller/SentryChassisController`
- Joint Mapping:
  - Steer: {{steer_joints}}
  - Wheel: {{wheel_joints}}

## Baseline PID
- Steer: {{baseline_steer}}
- Wheel: {{baseline_wheel}}
- Cost: {{baseline_cost}}

## Best PID
- Steer: {{best_steer}}
- Wheel: {{best_wheel}}
- Cost: {{best_cost}}
- Improvement: {{improvement_percent}}%

## Metrics
| Profile | ITAE | Settling Time | Overshoot Ratio | Steady-State Error | Weighted Cost |
|---|---:|---:|---:|---:|---:|
| Baseline/vx | {{baseline_vx_itae}} | {{baseline_vx_settling}} | {{baseline_vx_overshoot}} | {{baseline_vx_sse}} | {{baseline_vx_cost}} |
| Baseline/vy | {{baseline_vy_itae}} | {{baseline_vy_settling}} | {{baseline_vy_overshoot}} | {{baseline_vy_sse}} | {{baseline_vy_cost}} |
| Baseline/wz | {{baseline_wz_itae}} | {{baseline_wz_settling}} | {{baseline_wz_overshoot}} | {{baseline_wz_sse}} | {{baseline_wz_cost}} |
| Best/vx | {{best_vx_itae}} | {{best_vx_settling}} | {{best_vx_overshoot}} | {{best_vx_sse}} | {{best_vx_cost}} |
| Best/vy | {{best_vy_itae}} | {{best_vy_settling}} | {{best_vy_overshoot}} | {{best_vy_sse}} | {{best_vy_cost}} |
| Best/wz | {{best_wz_itae}} | {{best_wz_settling}} | {{best_wz_overshoot}} | {{best_wz_sse}} | {{best_wz_cost}} |

## Artifacts
- Summary JSON: `artifacts/pid_tuning/{{timestamp}}/summary.json`
- Trace CSV: `artifacts/pid_tuning/{{timestamp}}/traces.csv`
- Response Plots:
  - `artifacts/pid_tuning/{{timestamp}}/response_vx.png`
  - `artifacts/pid_tuning/{{timestamp}}/response_vy.png`
  - `artifacts/pid_tuning/{{timestamp}}/response_wz.png`

## Notes
- If improvement is below threshold, baseline parameters are retained.
- Final applied configuration backup:
  - `src/sentry_chassis_controller/config/chassis_controller.yaml.bak.{{timestamp}}`
