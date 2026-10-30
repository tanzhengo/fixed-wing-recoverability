# Manuscript

**Learning-Guided Terminal Continuation Search for Fixed-Wing Motion Planning**

- [Paper PDF](main.pdf)
- [Single editable LaTeX file](main.tex)
- [Chinese results and reproduction guide](../docs/recovery/FINAL_RESULTS.md)
- [Network, data and experiment evidence](../docs/recovery/learning/README.md)
- [Forest-density experiment and trajectory figure](../docs/recovery/forest_figure/README.md)

The English IEEE-format manuscript includes the actual trained neural network,
its feature representation and loss, separate training/validation/test maps,
matched-state budget comparisons, closed-loop evaluation, and input ablation.
It uses an anonymous author line for review; author and affiliation information
can be entered in main.tex when preparing a named submission.

## Edit and compile

All prose, equations, tables, captions and reference entries are embedded in
`main.tex`. Search for `BEGIN SECTION` or use the editor's section outline to
navigate. Reference entries are at `BEGIN REFERENCES`. Images remain in
`figures/`. The earlier split sources are retained in Git history.

In a LaTeX editor, select `main.tex` as the main document and XeLaTeX as the
compiler. Run XeLaTeX twice to resolve citations and cross-references. The
bibliography is embedded, so there is no BibTeX step.

The project contains every figure, table and bibliography needed to compile:

```bash
cd paper
tectonic --keep-logs main.tex
```

The delivered PDF uses Tectonic 0.17.0. A fresh installation may need network
access to obtain TeX packages. `IEEEtran.cls` is included with its original
copyright and license notices.

Create a standalone ZIP containing exactly one TeX file, the referenced
figures, the class file, a PDF preview and Chinese editing instructions:

```bash
python3 package_latex.py --output /path/to/manuscript.zip
```

## Revised evidence

The revision adds twelve matched training runs (State/Geometry × BCE/auxiliary
loss × three seeds), 512 validation execution episodes, and 2,304 closed-loop
episodes on 32 new test maps. Validation execution freezes the State+Aux model
before the new test split is opened. With the common RK4 prefix check, it
completes 118/128 tasks with zero recorded inflated-margin incursions;
Diverse completes 89 and Full 96 under the same check.

Exact compact export removes inactive inputs and the unused work output.
Deferred inference checks the common cheapest primary before constructing
learned priorities. Runtime comparisons use isolated, rotated repeated inputs;
component profiling is a separate pass. The full timing tables are in the PDF
and the derived runtime analysis.

Historical failure replay reproduces 512 archived episodes. A separate urban
matrix contains fifteen trials (five schedules × three library/gate settings).
The city planner reads supplied building boxes; lidar voxels document observed
space. The native Gazebo/RViz screenshots and original forest illustration are
retained as archived records, with their original models and settings explicit.

Protocols, hashes, raw observations, models and analysis scripts are indexed in
[revision/README.md](../docs/recovery/revision/README.md). The original frozen
models and prior test records remain in `docs/recovery/learning`.

Regenerate the current figures with `create_schematic.py`,
`create_revision_training_figure.py` and `create_revision_result_figures.py`.
The retained forest/city image files are sufficient for independent LaTeX
compilation; no ROS installation or experiment execution is needed to edit the
standalone ZIP. Avoid running the historical `create_learning_figures.py` to
update the revised manuscript: it targets the archived table/figure layout.

Figure 8 now uses a new acquisition flight with waypoints 2--5 at world XY
(-30,80), (70,200), (130,100), and (80,60). Heights and other settings are
unchanged. The flight completes the eight-waypoint cycle at 88.544 s.
The [waypoint update record](../docs/recovery/city_waypoint_update/README.md)
contains the evidence and exact figure command. Figure 7 and the fifteen
controlled urban trials retain the original route.
