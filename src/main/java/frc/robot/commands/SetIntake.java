// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntake extends  InstantCommand {

  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  @Override
  public void initialize() {
    addRequirements(intake);
  }

  @Override
  public void execute() {

      intake.setMotorPower(IntakeConstants.kPower);
  }
}
