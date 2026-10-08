// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class SystemConstants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  // TODO HARDWARE: verify USB assignments before competition.
  public static final int JOYSTICK_PORT = 0;
  public static final int XBOX_PORT = 1;
  public static final double XBOX_TRIGGER_THRESHOLD = 0.5;
  public static final OdometryMode ODOMETRY_MODE = OdometryMode.ODOMETRY_LIMELIGHT;

  /** Deploy-time localization selection. Camera loss always falls back to wheel/gyro odometry. */
  public enum OdometryMode {
    ODOMETRY_ONLY,
    ODOMETRY_LIMELIGHT
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
