# Manuscript

**Learning-Guided Terminal Continuation Search for Fixed-Wing Motion Planning**

- [Paper PDF](main.pdf)
- [LaTeX entry file](main.tex)
- [Chinese results and reproduction guide](../docs/recovery/FINAL_RESULTS.md)
- [Network, data and experiment evidence](../docs/recovery/learning/README.md)
- [Urban-building experiment and voxel figure](../docs/recovery/urban_figure/README.md)

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

An additional urban experiment contains 80 nominal episodes on eight paired
layouts. Twenty podium--tower buildings form the base layout, and eight infill
structures form the denser variant. Figure 6 uses the first seed, selected before
execution, and actual recorded flight states. Buildings are displayed as 1.5 m
voxels; analytic boxes with 1 m face margins enter both collision checking and
clearance features. The existing network weights remain frozen.

In Cost/Diverse/Learned/No-geometry/Full order, base-layout completion counts are
7/6/4/8/7 out of eight, and infill counts are 6/1/1/1/4. All outcomes are included.
Urban-run timing columns are diagnostic and are not pooled with the primary
isolated runtime measurements. The earlier forest study remains archived in
`docs/recovery/forest_figure`; the active manuscript uses the urban experiment.

Figures are regenerated from audited experiment outputs using
`create_schematic.py`, `create_learning_figures.py` and `create_urban_figure.py`.
The data-driven scripts expect the repository's `docs/recovery/learning` and
`docs/recovery/urban_figure` directories, respectively. Existing figure files suffice
for compilation of the standalone source package.

Historical nonlearning development experiments remain in docs/recovery/v3 and
v4. They are not merged with the new learning test split.
