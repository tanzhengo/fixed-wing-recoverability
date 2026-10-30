# Manuscript

**Learning-Guided Terminal Continuation Search for Fixed-Wing Motion Planning**

- [Paper PDF](main.pdf)
- [LaTeX entry file](main.tex)
- [Chinese results and reproduction guide](../docs/recovery/FINAL_RESULTS.md)
- [Network, data and experiment evidence](../docs/recovery/learning/README.md)

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

Figures are regenerated from audited experiment outputs using
`create_schematic.py` and `create_learning_figures.py`. The latter expects the
repository's `docs/recovery/learning` directory. Existing figure files suffice
for compilation of the standalone source package.

Historical nonlearning development experiments remain in docs/recovery/v3 and
v4. They are not merged with the new learning test split.
