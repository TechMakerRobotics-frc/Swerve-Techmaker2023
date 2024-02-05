// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntake extends SequentialCommandGroup {

  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  public SetIntake() {
    addCommands(
      new InstantCommand(()->intake.setMotorPower(IntakeConstants.kPower),intake)
    );
  }
}
