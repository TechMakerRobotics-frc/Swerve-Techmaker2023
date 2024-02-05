package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ResetIntake extends InstantCommand {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  public ResetIntake() {

    addRequirements(intake);
    
  }
  @Override
  public void initialize() {
    intake.setMotorPower(0);
  }
}
