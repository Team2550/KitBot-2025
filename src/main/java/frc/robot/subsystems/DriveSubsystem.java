// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final Spark leftLeader;
  private final Spark leftFollower;
  private final Spark rightLeader;
  private final Spark rightFollower;

  private final DifferentialDrive drive;

  public DriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new Spark(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new Spark(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new Spark(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new Spark(DriveConstants.RIGHT_FOLLOWER_ID);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    leftLeader.addFollower(leftFollower);
    rightLeader.addFollower(rightFollower);
  }

  @Override
  public void periodic() {
  }

  // Command to drive the robot with joystick inputs
  public Command driveArcade(
      DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
  }
}
