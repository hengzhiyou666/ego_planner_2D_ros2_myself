^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dog_ego_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-07-15)
------------------
* Reorganized the ROS 2 package around explicit planning, mapping, path-processing,
  and node responsibilities while retaining the package and executable names.
* Added frame-aware input handling, configurable topic interfaces, a Boolean goal
  status, stale-data checks, deterministic stop behavior, and compatibility aliases.
* Corrected grid, A*, path-progress, collision-checking, and B-spline correctness
  issues and added automated regression coverage.
* Prevented invalid B-spline initialization from reusing a previous task, added
  full-duration collision validation, and guaranteed endpoint sampling.
* Hardened isolated testing, point-cloud health checks, cloud/odometry time
  alignment, stale-odometry stop output, topic-loop validation, and safe path reuse.
* Added monotonic arc-length progress, timestamp/TF rollback guards, explicit
  planning states, standard unknown/free/occupied map encoding, and task-bound
  local-path reuse.
* Added isolated ROS graph integration tests for direct planning, obstacle stop,
  empty-path recovery, closed-loop goal handling, stale odometry, and TF failure.
* Replaced host-specific launch and build behavior with explicit launch arguments,
  a safe incremental build helper, and ROS 2 Humble continuous integration.
* Declared the combined work as GPL-3.0-only and documented EGO-Planner and
  LBFGS-Lite upstream licensing.
