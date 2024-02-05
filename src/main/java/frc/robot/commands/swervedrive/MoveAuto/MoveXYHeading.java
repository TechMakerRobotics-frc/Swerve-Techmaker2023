
package frc.robot.commands.swervedrive.MoveAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class MoveXYHeading extends CommandBase {
  
  double distanceX, distanceY, distanceH;
  SwerveSubsystem swerve;
  boolean finish = false;
  double lastTimestamp;
  double lastErrorX;
  double lastErrorY;
  double lastErrorH;
  public MoveXYHeading(double distanceX, double distanceY,double heading, SwerveSubsystem swerve) {
    
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    //this.distanceH = distanceH;
    
    distanceH = heading;
    SmartDashboard.putNumber("Distance Xi", distanceX);
    SmartDashboard.putNumber("Distance Yi", distanceY);
    SmartDashboard.putNumber("Distance Hi", distanceH);

    this.swerve = swerve;

  }
  @Override
  public void initialize() {

    swerve.resetOdometry();
    swerve.zeroGyro();
    lastTimestamp = Timer.getFPGATimestamp();
    lastErrorH = 0;
  }

  @Override
  public void execute() {


    double speedX = 0;
    double speedY = 0;
    double heading = 0;

    
    finish = true;
    if(Math.abs(swerve.getPose().getX())>Math.abs(distanceX)){
      finish = false;
    }
    if(Math.abs(swerve.getPose().getY())>Math.abs(distanceY)){
      finish = false;
    }
    if(Math.abs(swerve.getYaw().getDegrees())>Math.abs(distanceH)){
      finish = false;
    }

    // Cálculos -PID-
    double sensorX = swerve.getPose().getX();
    double errorX = distanceX - sensorX;
    speedX = Auton.kp*errorX;

    double sensorY = swerve.getPose().getY();
    double errorY = distanceY - sensorY;
    speedY = Auton.kp*errorY;

    double sensorH = swerve.getYaw().getDegrees();
    double errorH = distanceH - sensorH;
    heading = Auton.kpH*errorH;


    double errorSumX = 0;
    double errorSumY = 0;
    double errorSumH = 0;   

    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    double errorRateX = (errorX - lastErrorX) / dt;
    double errorRateY = (errorY - lastErrorY) / dt;
    double errorRateH = (errorH - lastErrorH) / dt;

    errorSumX += errorX * dt;
    errorSumY += errorY * dt;
    errorSumH += errorH * dt;

    speedX = Auton.kp * errorX + Auton.ki * errorSumX + Auton.kd * errorRateX;
    speedY = Auton.kp * errorY + Auton.ki * errorSumY + Auton.kd * errorRateY;
    heading = Auton.kpH * errorH + Auton.kiH * errorSumH + Auton.kdH * errorRateH;
 // Drive using raw values.
    swerve.drive(new Translation2d(speedX, speedY), heading , false ,false);
    lastTimestamp = Timer.getFPGATimestamp();

    lastErrorX = errorX;
    lastErrorY = errorY;
    lastErrorH = errorH;
    
  }

  
  @Override
  public void end(boolean interrupted) {
    swerve.lock();
  }

  // Retornar finish para terminar.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
