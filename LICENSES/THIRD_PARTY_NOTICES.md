# Third-party notices

This file records third-party provenance for `dog_ego_planner`. It does not replace the license text shipped with each component.

## EGO-Planner

Parts of the B-spline planning and optimization implementation are derived from [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner).

- Upstream project: EGO-Planner
- Upstream copyright: the EGO-Planner contributors
- Upstream license: GNU General Public License version 3
- Local license text: [`GPL-3.0-only.txt`](GPL-3.0-only.txt)
- Local modifications: ported to ROS 2 Humble and adapted from a 3D aerial-robot planner to a 2D quadruped local-planning workflow, with local mapping, path-processing, safety and interface changes

The adapted upstream implementation is stored under `third_party/ego_planner/` and in the planner integration sources. The mapping and A* modules under `src/` have been rewritten for this ROS 2 package rather than retained as legacy ROS 1 subpackages. Preserve this notice if the files are relocated.

## LBFGS-Lite

`third_party/lbfgs_lite/include/lbfgs.hpp` is derived from [ZJU-FAST-Lab/LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite).

- Copyright (c) 1990 Jorge Nocedal
- Copyright (c) 2007-2010 Naoaki Okazaki
- Copyright (c) 2020-2022 Zhepei Wang
- License: MIT
- Local license text: [`LBFGS-LITE-MIT.txt`](LBFGS-LITE-MIT.txt)

The MIT notice applies to the LBFGS-Lite component itself. Distribution as part of this combined planner remains subject to the GPL-3.0-only terms that govern the combined work.

## Project licensing

Except for separately identified third-party components, `dog_ego_planner` is distributed under GPL-3.0-only. See [`GPL-3.0-only.txt`](GPL-3.0-only.txt). No license is asserted here for code with unknown provenance; such code must be replaced or have its source and compatible license established before redistribution.
