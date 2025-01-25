// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;

public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(DriveSubsystem driveSubsystem, RollerSubsystem rollerSubsystem) {
    return driveSubsystem.driveArcade(driveSubsystem, () -> 0.5, () -> 0.0)
        .withTimeout(1.0)
        .andThen(rollerSubsystem.runRoller(rollerSubsystem, () -> 0.5, () -> 0));
  }
}
