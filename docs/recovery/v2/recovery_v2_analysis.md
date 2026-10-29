# Finite-continuation v2 experiment audit

Deterministic model-in-the-loop with shared planning/plant dynamics. Finite sampled continuations, no recursive-feasibility or physical-safety guarantee. Wall-clock planning cost is measured but not inserted as flight delay.

Timing repeats are collapsed per matched input. Development/validation are scenario labels; no independent generalization claim is implied.

## Closed-loop outcomes

| Split | Scenario | Variant | Episodes | Goals | Mean ms, pooled | P95 ms |
|---|---|---|---:|---:|---:|---:|
| development | cul_de_sac | baseline | 3 | 0 | 3.186 | 3.34 |
| development | cul_de_sac | cost12_h3 | 3 | 0 | 4.834 | 5.798 |
| development | cul_de_sac | cost12_h6 | 3 | 0 | 7.971 | 8.564 |
| development | cul_de_sac | cost203_h3 | 3 | 0 | 10.63 | 26.26 |
| development | cul_de_sac | cost203_h6 | 3 | 0 | 26.54 | 51.59 |
| development | cul_de_sac | diverse12_h3 | 3 | 0 | 5.013 | 5.872 |
| development | cul_de_sac | diverse12_h6 | 3 | 0 | 7.52 | 8.093 |
| development | cul_de_sac | diverse12_h6_cache | 3 | 0 | 7.487 | 8.074 |
| development | cul_de_sac | long6 | 3 | 0 | 6.068 | 6.173 |
| development | late_wall | baseline | 3 | 0 | 1.959 | 2.025 |
| development | late_wall | cost12_h3 | 3 | 0 | 3.439 | 3.688 |
| development | late_wall | cost12_h6 | 3 | 0 | 4.926 | 5.836 |
| development | late_wall | cost203_h3 | 3 | 3 | 2.533 | 5.201 |
| development | late_wall | cost203_h6 | 3 | 3 | 2.872 | 8.553 |
| development | late_wall | diverse12_h3 | 3 | 3 | 2.397 | 3.556 |
| development | late_wall | diverse12_h6 | 3 | 3 | 2.552 | 4.81 |
| development | late_wall | diverse12_h6_cache | 3 | 3 | 2.449 | 4.681 |
| development | late_wall | long6 | 3 | 3 | 4.034 | 4.375 |
| development | offset_pillars | baseline | 3 | 3 | 1.932 | 2.064 |
| development | offset_pillars | cost12_h3 | 3 | 3 | 2.171 | 2.911 |
| development | offset_pillars | cost12_h6 | 3 | 3 | 2.389 | 3.779 |
| development | offset_pillars | cost203_h3 | 3 | 3 | 2.176 | 2.926 |
| development | offset_pillars | cost203_h6 | 3 | 3 | 2.42 | 3.873 |
| development | offset_pillars | diverse12_h3 | 3 | 3 | 2.308 | 3.207 |
| development | offset_pillars | diverse12_h6 | 3 | 3 | 2.864 | 4.634 |
| development | offset_pillars | diverse12_h6_cache | 3 | 3 | 2.849 | 4.592 |
| development | offset_pillars | long6 | 3 | 2 | 3.905 | 4.437 |
| development | open | baseline | 3 | 3 | 1.598 | 1.762 |
| development | open | cost12_h3 | 3 | 3 | 1.708 | 1.821 |
| development | open | cost12_h6 | 3 | 3 | 1.69 | 1.739 |
| development | open | cost203_h3 | 3 | 3 | 1.714 | 1.84 |
| development | open | cost203_h6 | 3 | 3 | 1.702 | 1.855 |
| development | open | diverse12_h3 | 3 | 3 | 1.771 | 2.118 |
| development | open | diverse12_h6 | 3 | 3 | 1.754 | 1.881 |
| development | open | diverse12_h6_cache | 3 | 3 | 1.782 | 2.027 |
| development | open | long6 | 3 | 3 | 3.164 | 3.256 |
| robustness | mirrored_pillars | cost12_h6 | 4 | 4 | 2.887 | 5.219 |
| robustness | mirrored_pillars | cost203_h6 | 4 | 4 | 5.709 | 20.52 |
| robustness | mirrored_pillars | diverse12_h6 | 4 | 4 | 3.151 | 5.282 |
| robustness | mirrored_pillars | diverse12_h6_cache | 4 | 3 | 3.258 | 5.633 |
| robustness | wide_entry_cul_de_sac | cost12_h6 | 4 | 0 | 7.41 | 8.361 |
| robustness | wide_entry_cul_de_sac | cost203_h6 | 4 | 4 | 11.6 | 49.41 |
| robustness | wide_entry_cul_de_sac | diverse12_h6 | 4 | 3 | 4.391 | 7.675 |
| robustness | wide_entry_cul_de_sac | diverse12_h6_cache | 4 | 3 | 4.388 | 7.607 |
| validation | mirrored_pillars | baseline | 3 | 3 | 1.897 | 1.993 |
| validation | mirrored_pillars | cost12_h3 | 3 | 3 | 2.12 | 2.551 |
| validation | mirrored_pillars | cost12_h6 | 3 | 3 | 2.998 | 3.898 |
| validation | mirrored_pillars | cost203_h3 | 3 | 3 | 2.079 | 2.542 |
| validation | mirrored_pillars | cost203_h6 | 3 | 3 | 2.235 | 2.968 |
| validation | mirrored_pillars | diverse12_h3 | 3 | 3 | 2.188 | 3.043 |
| validation | mirrored_pillars | diverse12_h6 | 3 | 3 | 2.407 | 4.285 |
| validation | mirrored_pillars | diverse12_h6_cache | 3 | 3 | 2.399 | 4.322 |
| validation | mirrored_pillars | long6 | 3 | 3 | 3.775 | 4.086 |
| validation | wide_entry_cul_de_sac | baseline | 3 | 0 | 3.276 | 3.589 |
| validation | wide_entry_cul_de_sac | cost12_h3 | 3 | 0 | 5.575 | 18.33 |
| validation | wide_entry_cul_de_sac | cost12_h6 | 3 | 0 | 7.735 | 8.346 |
| validation | wide_entry_cul_de_sac | cost203_h3 | 3 | 0 | 8.636 | 21.48 |
| validation | wide_entry_cul_de_sac | cost203_h6 | 3 | 3 | 6.026 | 24.1 |
| validation | wide_entry_cul_de_sac | diverse12_h3 | 3 | 0 | 4.4 | 5.785 |
| validation | wide_entry_cul_de_sac | diverse12_h6 | 3 | 1 | 5.607 | 7.93 |
| validation | wide_entry_cul_de_sac | diverse12_h6_cache | 3 | 3 | 4.169 | 7.343 |
| validation | wide_entry_cul_de_sac | long6 | 3 | 0 | 6.029 | 6.468 |

The JSON also separates speed, observation radius, and backup horizon and reports all failure outcomes. Pooled cycle times compare different visited states; use the matched table for ranking efficiency.

## Matched-input finite-bank reference

| Split / scene | Backup s | Speed | Radius | Variant | Inputs | Reference positives | Recall | Median ms | Median step ratio |
|---|---:|---:|---:|---|---:|---:|---:|---:|---:|
| development / cul_de_sac | 3.0 | 15.5 | 0.0 | cost12 | 9 | 6 | 1 | 4.665 | 1 |
| development / cul_de_sac | 3.0 | 15.5 | 0.0 | cost203 | 9 | 6 | 1 | 4.663 | 1 |
| development / cul_de_sac | 3.0 | 15.5 | 0.0 | diverse12 | 9 | 6 | 1 | 5.401 | 1 |
| development / cul_de_sac | 6.0 | 15.5 | 0.0 | cost12 | 9 | 0 | — | 8.179 | 0.2946 |
| development / cul_de_sac | 6.0 | 15.5 | 0.0 | cost203 | 9 | 0 | — | 25 | 1 |
| development / cul_de_sac | 6.0 | 15.5 | 0.0 | diverse12 | 9 | 0 | — | 7.91 | 0.2946 |
| development / late_wall | 3.0 | 15.5 | 0.0 | cost12 | 9 | 6 | 0 | 3.62 | 0.8029 |
| development / late_wall | 3.0 | 15.5 | 0.0 | cost203 | 9 | 6 | 1 | 4.44 | 1 |
| development / late_wall | 3.0 | 15.5 | 0.0 | diverse12 | 9 | 6 | 1 | 3.295 | 0.6947 |
| development / late_wall | 6.0 | 15.5 | 0.0 | cost12 | 9 | 6 | 0 | 4.938 | 0.6928 |
| development / late_wall | 6.0 | 15.5 | 0.0 | cost203 | 9 | 6 | 1 | 7.298 | 1 |
| development / late_wall | 6.0 | 15.5 | 0.0 | diverse12 | 9 | 6 | 1 | 4.551 | 0.5984 |
| development / offset_pillars | 3.0 | 15.5 | 0.0 | cost12 | 9 | 7 | 1 | 2.091 | 1 |
| development / offset_pillars | 3.0 | 15.5 | 0.0 | cost203 | 9 | 7 | 1 | 2.08 | 1 |
| development / offset_pillars | 3.0 | 15.5 | 0.0 | diverse12 | 9 | 7 | 1 | 2.108 | 1 |
| development / offset_pillars | 6.0 | 15.5 | 0.0 | cost12 | 9 | 7 | 1 | 2.015 | 1 |
| development / offset_pillars | 6.0 | 15.5 | 0.0 | cost203 | 9 | 7 | 1 | 2.127 | 1 |
| development / offset_pillars | 6.0 | 15.5 | 0.0 | diverse12 | 9 | 7 | 1 | 2.081 | 1 |
| development / open | 3.0 | 15.5 | 0.0 | cost12 | 9 | 9 | 1 | 1.676 | 1 |
| development / open | 3.0 | 15.5 | 0.0 | cost203 | 9 | 9 | 1 | 1.677 | 1 |
| development / open | 3.0 | 15.5 | 0.0 | diverse12 | 9 | 9 | 1 | 1.712 | 1 |
| development / open | 6.0 | 15.5 | 0.0 | cost12 | 9 | 9 | 1 | 1.684 | 1 |
| development / open | 6.0 | 15.5 | 0.0 | cost203 | 9 | 9 | 1 | 1.683 | 1 |
| development / open | 6.0 | 15.5 | 0.0 | diverse12 | 9 | 9 | 1 | 1.72 | 1 |
| validation / mirrored_pillars | 3.0 | 15.5 | 0.0 | cost12 | 9 | 9 | 1 | 2.22 | 1 |
| validation / mirrored_pillars | 3.0 | 15.5 | 0.0 | cost203 | 9 | 9 | 1 | 2.196 | 1 |
| validation / mirrored_pillars | 3.0 | 15.5 | 0.0 | diverse12 | 9 | 9 | 1 | 2.209 | 1 |
| validation / mirrored_pillars | 6.0 | 15.5 | 0.0 | cost12 | 9 | 9 | 1 | 3.04 | 1 |
| validation / mirrored_pillars | 6.0 | 15.5 | 0.0 | cost203 | 9 | 9 | 1 | 2.992 | 1 |
| validation / mirrored_pillars | 6.0 | 15.5 | 0.0 | diverse12 | 9 | 9 | 1 | 4.027 | 1 |
| validation / wide_entry_cul_de_sac | 3.0 | 15.5 | 0.0 | cost12 | 9 | 9 | 1 | 3.468 | 1 |
| validation / wide_entry_cul_de_sac | 3.0 | 15.5 | 0.0 | cost203 | 9 | 9 | 1 | 3.514 | 1 |
| validation / wide_entry_cul_de_sac | 3.0 | 15.5 | 0.0 | diverse12 | 9 | 9 | 1 | 3.475 | 1 |
| validation / wide_entry_cul_de_sac | 6.0 | 15.5 | 0.0 | cost12 | 9 | 6 | 0.3333 | 8.196 | 0.348 |
| validation / wide_entry_cul_de_sac | 6.0 | 15.5 | 0.0 | cost203 | 9 | 6 | 1 | 22.22 | 1 |
| validation / wide_entry_cul_de_sac | 6.0 | 15.5 | 0.0 | diverse12 | 9 | 6 | 0.5 | 7.844 | 0.348 |

Recall is conditional on cost203 finding a finite-library continuation; it is not physical safety recall. Zero reference positives produces an undefined recall, not zero or one.

## Audit findings

No schema, pairing, repeatability, or finite-bank consistency problems detected.
