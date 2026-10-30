# Manuscript

**Learning-Guided Terminal Continuation Search for Fixed-Wing Motion Planning**

- [Paper PDF](main.pdf)
- [LaTeX entry file](main.tex)
- [Chinese results and reproduction guide](../docs/recovery/FINAL_RESULTS.md)
- [Network, data and experiment evidence](../docs/recovery/learning/README.md)
- [Forest-density experiment and trajectory figure](../docs/recovery/forest_figure/README.md)

The English IEEE-format manuscript includes the actual trained neural network,
its feature representation and loss, separate training/validation/test maps,
matched-state budget comparisons, closed-loop evaluation, and input ablation.
It uses an anonymous author line for review; author and affiliation information
can be entered in main.tex when preparing a named submission.

## Compile

The project contains every figure, table and bibliography needed to compile:

```bash
cd paper
tectonic --keep-logs main.tex
```

The delivered PDF uses Tectonic 0.17.0. A fresh installation may need network
access to obtain TeX packages. The generated main.bbl is included.

## Evidence

The primary learning suite contains 576 matched states (11 methods/budgets,
two timing repeats) and 768 closed-loop method-map-condition episodes. A
separate frozen-model geometry-input ablation adds 256 changed-method episodes
on the same maps and replays 512 unchanged baseline episodes exactly. These
replays do not add independent maps. The 32 test maps are separate from the
160 training and 32 validation maps.

The prospectively frozen geometry model completes 99/128 tasks; the subsequent
no-geometry input ablation completes 117/128. The manuscript retains both
results and their different experimental roles. The primary runtime timings
were collected independently of training and compilation.

An additional forest experiment contains 80 nominal episodes on eight paired
sparse/dense seeds. The two-density trajectory figure uses the first seed,
specified before execution, and actual logged flight states. The obstacle display
uses a 1.5 m occupancy grid derived from the analytic geometry; collision checks
retain the original analytic primitives. All methods finish
8/8 sparse tasks; in dense forests, Learned finishes 7/8 and the other four
methods finish 8/8. The displayed dense panel includes the Learned margin
termination. Forest-run timing columns are diagnostic and are not pooled with
the primary isolated runtime measurements.

Figures are regenerated from audited experiment outputs using
`create_schematic.py`, `create_learning_figures.py` and `create_forest_figure.py`.
The data-driven scripts expect the repository's `docs/recovery/learning` and
`docs/recovery/forest_figure` directories, respectively. Existing figure files suffice
for compilation of the standalone source package.

Historical nonlearning development experiments remain in docs/recovery/v3 and
v4. They are not merged with the new learning test split.
