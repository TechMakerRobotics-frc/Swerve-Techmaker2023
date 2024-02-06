
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ResetShoot extends InstantCommand {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  public ResetShoot() {

    addRequirements(intake);
    addRequirements(shooter);

    
  }

  @Override
  public void initialize() {
    shooter.setMotorPower(0);
    intake.setMotorPower(0);
  }
}
