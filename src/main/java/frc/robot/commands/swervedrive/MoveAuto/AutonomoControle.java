
package frc.robot.commands.swervedrive.MoveAuto;


import frc.robot.commands.Auto.AutonomosProntos.AutoDiagonal;
import frc.robot.commands.Auto.AutonomosProntos.AutoQuadrado;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomoControle extends SequentialCommandGroup {

  Command defaultDriveCommand;
  public AutonomoControle (SwerveSubsystem drivebase){
  {
    drivebase.removeDefaultCommand(); // Provavelmente isso está atrapalhando.
      addCommands(

  new AutoQuadrado(drivebase),
  new AutoDiagonal(drivebase));
  
  // para andar 2metros x = 2.1
  // para andar 2metros y = 2.2


  //drivebase.setDefaultCommand();  Talvez colocar isso.
    }
  }
}
