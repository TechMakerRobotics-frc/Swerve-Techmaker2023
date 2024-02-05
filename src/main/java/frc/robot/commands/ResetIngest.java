package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class ResetIngest extends InstantCommand {
  private final Intake intake = Intake.getInstance();

  public ResetIngest() {

    addRequirements(intake);
    
  }
  @Override
  public void initialize() {
    intake.setMotorPower(0);
  }
}
