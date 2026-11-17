# Illustrated architecture figure

This directory renders Figure 1 from the bundled Gazebo and RViz screenshot crops. It does not run planning experiments or modify manuscript files.

Install the Python dependencies with `python3 -m pip install -r requirements.txt`, then run:

```sh
python3 create_architecture.py --font-dir /path/to/times-new-roman --output /tmp/figure1
```

The font directory must contain `times.ttf`, `timesbd.ttf` and `timesi.ttf` (names are case-insensitive). Actual Times New Roman fonts are required; fonts are not distributed here and no substitutes are selected. The regular, bold and italic faces are embedded in the generated PDF and SVG.

Outputs are `architecture_revised.pdf`, `architecture_revised.svg` and `figure_design.json`. The SVG retains editable text and vector geometry and embeds its two screenshot crops. The default output directory is `generated/` beside the script. To obtain a PNG with Poppler:

```sh
pdftoppm -png -singlefile -r 216 /tmp/figure1/architecture_revised.pdf /tmp/figure1/architecture_revised
```

The figure uses the manuscript configuration: 58 input features, H=B=3 s, Selected BCE with auxiliary-work supervision as an ablation, the common first query charged to K, fixed-schedule cost pruning without refilling, and dispatch/remaining-sequence revalidation. The aircraft and voxel images are project simulation acquisitions; the motion-primitive fans and networks are schematic.

`figure_design.json` records source provenance and label placement; `QA.json` records the visual checks of the original paper figure. `portability_check.json` records the portable-source verification. No original desktop screenshot paths are needed to regenerate the figure.
