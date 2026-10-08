# Localization AdvantageScope validation

All items below require user validation in SIM; no physical Limelight or AprilTags are needed.

| Test | Setup and signals | Expected result |
| --- | --- | --- |
| V01 odometry-only motion | Set `ODOMETRY_MODE` to `ODOMETRY_ONLY`; show `Drive/Pose` on the 2026 REBUILT field; drive forward/back/strafe/rotate. | Motion is coherent; no vision correction. |
| V02 fusion | Set `ODOMETRY_LIMELIGHT`; show `Drive/Pose`, `Vision/AcceptedPoses`, `Vision/RejectedPoses`, `Vision/Connected`; inject an odometry offset and valid sim observation. | `Drive/Pose` moves smoothly toward vision, without a reset teleport. |
| V03 disconnect | Move, then set `VisionIOSim.connected` false. | `Drive/Pose` continues, remains finite, and does not jump to zero. |
| V04 reconnect | After V03, reconnect and submit a valid observation. | Fusion resumes with no restart, estimator recreation, or mode change. |
| V05 mode log | Inspect `Drive/OdometryMode` during both modes and reconnect. | It remains the configured enum. |
| V06 accepted/rejected | Show accepted/rejected poses and counts; inject valid, zero-tag, out-of-field, and bad-Z observations. | Only valid poses are accepted. |
| V07 MegaTag1 | Inject `MEGATAG_1`; show `Vision/MegaTag1/RobotPose` and `Drive/Pose`. | The same final drive pose responds. |
| V08 MegaTag2 | Inject `MEGATAG_2`; show heading, `Vision/MegaTag2/RobotPose`, and `Drive/Pose`. | MT2 uses drivetrain heading and corrects the same final pose. |
| V09 final pose | Use `Drive/Pose` for the field robot. | MegaTag pose keys remain diagnostics only. |
| V10 coordinates | On 2026 REBUILT, command forward, strafe, rotate 90 degrees, then field-relative drive. | No X/Y swap or 90-degree model offset. |
| V11 continuity | Mix translation, rotation, and vision corrections. | No unexplained teleports, heading flips, NaN, or Infinity. |
| V12 rejected while moving | Drive while sending rejected samples. | Odometry continues normally. |
| V13 confidence | Compare near/multi-tag and far/single-tag samples using `Vision/LinearStdDev` and `Vision/AngularStdDev`. | Higher-confidence samples show lower uncertainty. |
| V14 SIM-only | Run with no Limelight/CANcoder/Pigeon/SparkMax hardware. | VisionIOSim, ModuleIOSim, and GyroIOSim operate normally. |

Available diagnostics: `Drive/Pose`, `Drive/OdometryMode`, `Vision/Connected`, `Vision/Mode`, `Vision/TagIds`, `Vision/ObservationCount`, `Vision/AcceptedCount`, `Vision/RejectedCount`, `Vision/AcceptedPoses`, `Vision/RejectedPoses`, `Vision/MegaTag1/RobotPose`, `Vision/MegaTag2/RobotPose`, `Vision/LinearStdDev`, and `Vision/AngularStdDev`.
