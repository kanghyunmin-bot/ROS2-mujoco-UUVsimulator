# BlueROV2 figures and replay evidence

| Figure | Content | Files |
| --- | --- | --- |
| 01 | Initial and yaw-tuned UUV-Sim replay against tank measurements | [PDF](figure-01-initial-final.pdf) · [PNG](figure-01-initial-final.png) · [RMSE](data/figure-01-initial-final-rmse.csv) · [Aligned series](data/figure-01-initial-final-timeseries.csv) |
| 02 | UUV-Sim and public Stonefish and Gazebo model replays | [PDF](figure-02-simulator-comparison.pdf) · [PNG](figure-02-simulator-comparison.png) · [RMSE](data/figure-02-simulator-comparison-rmse.csv) · [Aligned series](data/figure-02-simulator-comparison-timeseries.csv) |
| 03 | Bounded least-squares workflow for the 6-DOF model | [PDF](figure-03-calibration-workflow.pdf) · [PNG](figure-03-calibration-workflow.png) · [Editable draw.io](figure-03-calibration-workflow.drawio) |
| 04 | MuJoCo, Python runtime and ROS 2 interfaces | [PDF](figure-04-mujoco-framework.pdf) · [PNG](../assets/mujoco-framework.png) · [SVG](figure-04-mujoco-framework.svg) · [Editable PPTX](figure-04-mujoco-framework.pptx) |
| Ablation | Original calibration versus zero lateral center-of-buoyancy offset | [PNG](cob-lateral-comparison.png) · [RMSE](data/cob-lateral-rmse.csv) |

Figures 01 and 02 use the DFKI random-path trial and pair valid measurements and simulations at 10 Hz. Attitude uses the original attitude timestamps. Gaps in surge, sway, and relative depth remain visible. Both figures use the same absolute time grid. Figure 01 uses the first measured timestamp (0.01199675 s) as the reference for relative depth and accumulated yaw. Figure 02 uses the first timestamp shared with Gazebo (0.11199675 s).

The random-path trial was used to fit UUV-Sim. Stonefish and Gazebo use their public BlueROV2 models and individual actuator conversions and tank settings. Figure 02 compares those particular replays. The general DFKI profile is `bluerov2_dfki_calibrated`. The optional experimental yaw fit is the red curve in the figures and is the current interactive GUI preset. Its pooled accumulated yaw error increased on fitting-excluded trials. The lateral CoB ablation compares the same yaw-tuned profile with and without the lateral offset, using the same recorded PWM and voltage.

## Reproduce or replay

- Run `python docs/paper/render_figures.py` with NumPy and Matplotlib to regenerate Figures 01 and 02. The script verifies the reported RMSE against the aligned series.
- See the [BlueROV2 model and replay commands](../contracts/BLUEROV2_MODEL.md) for raw-data preparation, plant replay, and scoring. Download the original dataset from [Zenodo v3](https://zenodo.org/records/17360027).
- For the web GUI's **RC Log Replay**, the bundled [DFKI random RC input bag](data/dfki-random-rcin-bag/) is available at `/workspace/docs/paper/data/dfki-random-rcin-bag` in the Docker runtime. It converts the first six channels from the public random-path `rc-in.csv` into `/mavros/rc/override`, preserves sample intervals, and leaves auxiliary channels untouched. The GUI replays pilot inputs through SITL. Figures 01 and 02 use recorded ESC outputs and the UUV-Sim replay uses the recorded voltage trace.
- Figure 03 can be exported with `python docs/paper/render_calibration_workflow.py` using draw.io Desktop, Ghostscript, Poppler and pypdf. Figure 04 can be edited in PowerPoint or regenerated using [its renderer](render_framework.mjs), the bundled logo inputs, the Codex artifact runtime, Chrome and Poppler.

## Sources and credits

- Aligned measurements and the example traces derive from the BlueROV2 tank dataset by Juri Khanmeh, Bilal Wehbe, Enrico Simetti, and Giovanni Indiveri, licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The figures align, transform and subset the published samples. [Zenodo v3](https://doi.org/10.5281/zenodo.17360027) · [Data paper](https://doi.org/10.1038/s41597-025-06347-0)
- The converted RC input bag uses the same authors' [Zenodo v1.1 archive](https://doi.org/10.5281/zenodo.15976877), also under CC BY 4.0.
- Figure 03 product photograph courtesy of [Blue Robotics](https://bluerobotics.com/press/). Figure 04 marks are from [MuJoCo](https://mujoco.org/), the [Python Software Foundation](https://www.python.org/community/logos/), and [Open Robotics](https://www.ros.org/blog/media/).
- See the [model, data, and comparison sources](../../uuv_mujoco/current/research_workspace/04_paper_evidence/REFERENCES_BLUEROV2.md) and [BibTeX](../../uuv_mujoco/current/research_workspace/04_paper_evidence/references_bluerov2.bib).
