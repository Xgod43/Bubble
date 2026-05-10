# Bubble 2D Tactile Workspace

This folder is a separate working copy for the Pi Camera V3 NoIR / 2D tactile
contact-map experiments.

Original workspace left untouched:

`/home/bubble/Desktop/VespaBattleCatFC/Bubble`

Use this copy for the next camera pipeline changes:

`/home/bubble/Desktop/VespaBattleCatFC/Bubble_2D_Tactile_Workspace`

## Current contact/depth behavior

- Live Detection now starts the pressure sensor automatically when pressure
  contact gating is enabled.
- The pressure gate captures a baseline first, then holds camera contact/depth
  at zero until pressure rises above the gate threshold.
- On first pressure contact, the camera contact-map zero is captured from the
  first contact frames, then deformation/depth starts from there.
- Stepper depth is disabled by default while the motor is unavailable and shows
  `unavailable (manual)`.

Useful environment overrides:

```bash
BUBBLE_PRESSURE_CONTACT_DELTA_HPA=1.5 python3 launch_bubble_app.py
BUBBLE_PRESSURE_CONTACT_GATE=0 python3 launch_bubble_app.py
BUBBLE_STEPPER_DEPTH_ENABLED=1 BUBBLE_STEPPER_MM_PER_REV=8.0 python3 launch_bubble_app.py
```
