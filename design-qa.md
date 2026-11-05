# UT27 research workspace design QA — 2026-11-03

final result: passed

## Scope and visual target

The user selected reference 1 (Rerun) and requested an adaptation of the existing simulation GUI, opened in Chrome. This is a layout and appearance adaptation, not a pixel-identical clone or a migration to the Rerun SDK.

Source visual truth: https://static.rerun.io/viewer_walkthrough_open/e152be01e1d6ceeb774ddccbc49844430626100f/full.png

Source capture: `/home/khm/.codex/visualizations/2026/11/03/01a19f5c-36c1-73f1-8efe-c9261b7ab444/rerun-source-workspace.png`

Implementation: `http://127.0.0.1:18879/`

Implementation screenshot: `/home/khm/.codex/visualizations/2026/11/03/01a19f5c-36c1-73f1-8efe-c9261b7ab444/ut27-rerun-workspace.png`

Full-view and focused comparison in the same image: `/home/khm/.codex/visualizations/2026/11/03/01a19f5c-36c1-73f1-8efe-c9261b7ab444/ut27-rerun-comparison.png`

Narrow-layout screenshot: `/home/khm/.codex/visualizations/2026/11/03/01a19f5c-36c1-73f1-8efe-c9261b7ab444/ut27-rerun-mobile.png`

## Evidence and normalization

- Source screenshot pixels: 1280 × 720; underlying source image: 3588 × 2258, displayed with aspect ratio preserved.
- Final implementation screenshot pixels and inspected CSS viewport: 1600 × 900. Browser reported devicePixelRatio 1.6; saved screenshot dimensions were used for normalization rather than assuming physical pixels from DPR.
- Both full screenshots normalized to 800 × 450 in the combined comparison. The lower pair enlarges the top-left chrome/navigation from copies normalized to 1600 × 900.
- Responsive check at inspected CSS viewport 487 × 1055. No horizontal overflow; header bottom 88.1 px, navigation top 96.1 px after correction.
- Source state: recorded computer-vision dataset with side panels and timeline. Implementation state: explicitly labelled read-only preview with two existing saved UUV simulation frames. Sensors remain unavailable rather than showing invented readings.
- Responsive and interaction verification used the in-app browser. The final preview was also opened and visually verified in native Chrome, and its tab was retained for the user. Chrome capture: `/home/khm/.codex/visualizations/2026/11/03/01a19f5c-36c1-73f1-8efe-c9261b7ab444/ut27-rerun-chrome.png` (964 × 516). At this compact window size, the observation and inspector scroll independently. Chrome logs contained two extension-origin connection errors, but no application-origin errors.

## Comparison history

1. P1 composition failure during initial assembly: detached controls could not be found while reparenting. Attached the workspace before ID lookups and left controls connected until they entered the inspector. Fresh-page verification has no console errors and no duplicate IDs.
2. P2 desktop header clipping: inherited grid layout wrapped the status/command row beyond a fixed header. Explicit flex layout corrected it. Final desktop screenshot shows the complete header.
3. P2 narrow header overlap: fixed header height allowed its second row to overlap navigation. At the small breakpoint, header height now grows with content. Rechecked bounding rectangles and saved the narrow screenshot.
4. P2 narrow inspector access: added an explicit close button inside the inspector and default collapse on narrow initial loads. Open/close verified.
5. Removed responsive hiding of the sensor summary so the data remains reachable at narrow widths. It stacks beneath the two instruments on mobile.

## Required fidelity surfaces

- Typography: system sans-serif for Korean labels and navigation, monospace for measured values and mode names. Intentionally larger controls than the dense Rerun developer tool; no missing text or clipped labels in verified states.
- Spacing/layout: left workspace navigation, dominant center observation, right inspector, bottom replay. Thin dividers and consistent 10–12 px panel spacing. Right inspector collapses to expand the center.
- Colors/tokens: neutral charcoal chrome and restrained blue selection. Warning and stop colors remain limited to their action semantics. Preview command controls are disabled intentionally.
- Image quality/assets: real saved simulation frames, original aspect ratio preserved. Local Feather 4.29.2 icons with MIT license, no network/runtime icon dependency. No fabricated 3-D robot scene or fabricated telemetry.
- Copy/content: UT27 experiment, collection, settings, and control labels. Replay explicitly says it sends RC commands; it is not labelled as video seeking. The preview badge and camera caption distinguish saved images from live operation.

## Interactions verified

- All/front/hand camera layout selection.
- Workspace navigation to observation, data/policy, and experiment settings.
- Inspector hide/show and mobile close.
- Settings popup, contextual help, Escape dismissal, and inert background.
- Collection popup moves the existing camera and pilot elements; closing restores both.
- Camera expansion and Escape restore camera and pilot to their original containers.
- No duplicate DOM IDs, no final-page console errors, no horizontal page overflow.
- Existing six JavaScript test files pass (preview pacing/adaptation, gamepad calibration, RC timeout/keepalive, recorder progress, sensor display, sensor-error selection).
- JavaScript syntax checks and `git diff --check` pass.

## Intentional differences and remaining limits

- Main content uses the existing front/hand camera feeds. A web 3-D scene endpoint is not currently implemented; native MuJoCo scene rendering was not added in this visual pass.
- The bottom dock reuses existing command replay. General synchronized sensor/video replay and scientific comparison plots remain future functionality.
- Live ROS/SITL control and recording were not executed during this design pass. Preview server rejects all POST commands with 405.
- Preview-only server and saved frames live under `output/ut27-workspace-preview/`; the real GUI HTML does not include preview locks or saved-frame data.
- P3: lower-level legacy diagnostics retain some English labels and historical panel styling.
