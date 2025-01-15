// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Class to run the rollers over CAN */
public class RollerSubsystem extends SubsystemBase {
  private final Spark rollerMotor;

  public RollerSubsystem() {
    // Set up the roller motor as a brushed motor
    rollerMotor = new Spark(RollerConstants.ROLLER_MOTOR_ID);
  }

  @Override
  public void periodic() {
  }

  // Command to run the roller with joystick inputs
  public Command runRoller(
      RollerSubsystem rollerSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(
        () -> rollerMotor.set(forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
  }

}
