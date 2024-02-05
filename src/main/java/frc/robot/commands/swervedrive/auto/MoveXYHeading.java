
package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class MoveXYHeading extends CommandBase {

  double distanceX, distanceY, heading;
  SwerveSubsystem swerve;
  PIDController pidX, pidY, pidHeading;

  public MoveXYHeading(double distanceX, double distanceY, double heading, SwerveSubsystem swerve) {

    this.distanceX = distanceX;
    this.distanceY = distanceY;
    this.heading = heading;
    pidX = new PIDController(Auton.kp, Auton.ki, Auton.kd);
    pidY = new PIDController(Auton.kp, Auton.ki, Auton.kd);
    pidX.setSetpoint(distanceX);
    pidY.setSetpoint(distanceY);
    this.swerve = swerve;
    addRequirements(swerve);

  }

  @Override
  public void initialize() {

    swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
    swerve.zeroGyro();
  }

  @Override
  public void execute() {

    double speedX = pidX.calculate(swerve.getPose().getX());
    double speedY = pidY.calculate(swerve.getPose().getY());
  
    swerve.drive(new Translation2d(speedX, speedY), heading, true, false);
   
  }

  @Override
  public void end(boolean interrupted) {
    swerve.lock();
  }

  // Retornar finish para terminar.
  @Override
  public boolean isFinished() {
    if (Math.abs(swerve.getYaw().getDegrees()) > Math.abs(heading)) {
      return false;
    }
    if(pidX.atSetpoint() && pidY.atSetpoint()){
      return true;
    }
    return false;
  }
}
