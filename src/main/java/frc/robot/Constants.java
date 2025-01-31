// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.Conversions;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 0;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 2;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class CoralHandlerConstants {
    public static final double restPose = 0;
    public static final double L2Pose = 0.3;
    public static final double L3Pose = 0.762;
    public static final double L4Pose = 0;  // Set this

    public static final Rotation2d armRestPose = new Rotation2d(0*Math.PI/180);
    public static final Rotation2d R2Pose = new Rotation2d(90*Math.PI/180);
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 4;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
