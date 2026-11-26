# Simulation verification record for 2026-11-26

The reported runs used Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, SLAM Toolbox and Nav2. This record summarizes the historical runs. The ROS package contains the current launch files, configurations and executable scripts.

## Frontier exploration

The run used the test room, EKF odometry TF, an asynchronous SLAM Toolbox map, Navfn tolerance of `0.1 m`, and a path endpoint threshold of `0.1 m`. Three goals succeeded and one aborted out of four `NavigateToPose` requests. Path prechecks rejected multiple candidates before navigation. These totals apply to a single run.

The original snapshot was checked locally with `sha256sum -c SHA256SUMS`. All 11 entries passed. Before release edits, its runtime files matched the primary package byte for byte except `config/nav2_params.yaml`, which parsed to the same values. The release uses a package-relative default in the localization launch and adds source change notices. The frozen source archive and its checksum manifest remain with the original local experiment. The released [PGM](../maps/frontier_explore_v1.pgm) and [YAML](../maps/frontier_explore_v1.yaml) are byte-identical to the checked snapshot. No additional Gazebo or RViz screenshots were available for this run.

The image is 239 × 239 at 0.05 m/cell. It contains 53,522 free, 2,107 occupied and 1,492 unknown pixels in the trinary map. Counting known pixels over the full grid yields 97.39%. A free cell adjoining an unknown cell in its eight-neighborhood is counted as a frontier, with the outermost rows and columns skipped. This gives 30 frontier cells in 12 eight-connected components.

## EKF

The frozen EKF parameters and covariance relay matched the package byte for byte before release editing. The release adds a provenance comment to the EKF YAML. The relay preserves the wheel odometry’s lateral velocity and supplies a variance of `1.0e-5`. The planar EKF selects wheel forward and lateral velocity and yaw rate, plus IMU yaw rate.

The recorded 60-second stationary check reported no recurrence of extreme lateral covariance divergence. In a short backward move, Gazebo odometry reported −0.101525 m and EKF odometry reported −0.101032 m, a difference of 0.000493 m. In a rotation check, the readings were 22.4198° and 22.5586°, a difference of 0.1388°. Gazebo odometry is a reference input to the comparison. These checks establish short-term consistency in the recorded run. Independent ground truth and long-term accuracy were outside the experiment.
