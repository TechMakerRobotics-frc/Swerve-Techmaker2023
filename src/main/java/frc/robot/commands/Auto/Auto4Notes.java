
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetIntake;
import frc.robot.commands.IntakeSensor;
import frc.robot.commands.ResetIntake;
import frc.robot.commands.swervedrive.auto.MoveXYHeading;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto4Notes extends SequentialCommandGroup {

  public Auto4Notes(SwerveSubsystem drivebase) {

    addCommands(
      new IntakeSensor(),
      new MoveXYHeading(-2.1, 0, 0, drivebase),
      new ResetIntake(),
      new IntakeSensor(),
      new MoveXYHeading(-1, 0, 0, drivebase),
      new ResetIntake()
    
    );
  }
}