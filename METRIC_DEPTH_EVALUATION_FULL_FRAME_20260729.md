# UniDepth V2 calibration evaluation

- Ready: `True`
- Reason: `ready`
- Total/valid rows: 5246/4950
- Calibration/validation: 4872/78
- Selected profile: `scale`
- Production recommended: `False`
- Physically rejected profiles: `{'affine': 'non_positive_scale', 'piecewise_linear': 'non_monotonic_mapping'}`

| Mode | MAE | RMSE | Bias | AbsRel | P95 |
|---|---:|---:|---:|---:|---:|
| legacy_center_median | 7.4528 | 8.2073 | 7.4528 | 0.8009 | 12.1063 |
| none | 8.0928 | 8.7991 | 8.0928 | 0.8641 | 12.7560 |
| scale | 2.3651 | 3.2868 | 1.5220 | 0.2779 | 5.4457 |
| affine | 2.1851 | 2.4300 | -2.1851 | 0.1935 | 2.9800 |
| piecewise_linear | 2.6079 | 3.8603 | 1.9941 | 0.3130 | 6.4540 |

| Filter | MAE | RMSE | Boundary mismatch | Mean delay ms |
|---|---:|---:|---:|---:|
| none | 4.9592 | 5.0726 | 0 | 0.00 |
| ema_0_30 | 4.9609 | 5.0717 | 10 | 691.31 |
| ema_0_65 | 4.9598 | 5.0726 | 3 | 159.53 |
| median3 | 4.9593 | 5.0723 | 4 | 0.00 |
| confidence_weighted_ema | 4.9604 | 5.0725 | 5 | 269.01 |
