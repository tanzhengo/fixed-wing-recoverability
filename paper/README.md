# Manuscript

**Maneuver-Diverse Terminal Continuation Search for Fixed-Wing Local Motion Planning**

- [Paper PDF](main.pdf)
- [LaTeX entry file](main.tex)
- [Chinese results and reproduction guide](../docs/recovery/FINAL_RESULTS.md)

This complete English research draft uses the IEEE two-column journal layout
and an anonymous author line. The paper includes the problem formulation,
algorithm, prediction model, controlled experiments, uncertainty estimates,
failure analysis, discussion, and references. Author and affiliation details
can be supplied when preparing a named submission.

## Compile

The project is self-contained: all figures, tables, bibliography, and section
sources are inside this directory. With Tectonic installed:

```bash
cd paper
tectonic --keep-logs main.tex
```

The delivered PDF was compiled with Tectonic 0.17.0. A fresh installation may
need network access to retrieve TeX packages and fonts. The included generated
`main.bbl` retains the formatted bibliography. A conventional LaTeX installation
can also compile the sources with XeLaTeX and BibTeX.

## Experiment provenance

The complete-rollout experiment is recorded at source commit `4b3b0f6` and in
[`docs/recovery/v3/manifest.json`](../docs/recovery/v3/manifest.json).
The incremental-rejection results are in
[`docs/recovery/v4/`](../docs/recovery/v4/).

The main task matrix contains 480 method–map–condition episodes. Repeating it
with the incremental validator reproduces every outcome and recorded state.
Each validator also has 6,048 matched-input observations (432 states × 7
methods × 2 timing repeats). Timing repeats and implementation replays are
not additional independent maps.

The benchmark and analysis scripts regenerate the empirical figures. The
schematic can be regenerated with `python3 create_schematic.py`. The source
notes for the references are preserved in `literature_sources.md`; they
separate paper content verification from bibliographic metadata checks.
