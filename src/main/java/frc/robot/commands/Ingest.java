// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class Ingest extends SequentialCommandGroup {

  private final Intake intake = Intake.getInstance();

  public Ingest() {
    addCommands(
      new InstantCommand(()->intake.setMotorPower(IntakeConstants.kPower),intake)
    );
  }
}
