// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;
import frc.robot.Constants.Auton;

public class MoveXY extends CommandBase {
  /** Creates a new MoveStraight. */
  double distanceX, distanceY;
  SwerveSubsystem swerve;
  boolean finish = false;
  private final SwerveController controller;
  double lastTimestamp;
  double lastErrorX = 0;
  double lastErrorY = 0;
  double errorSumX = 0;
    double errorSumY = 0;
  public MoveXY(double distanceX, double distanceY, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distanceX = distanceX;
    this.distanceY = distanceY;
    SmartDashboard.putNumber("Distance Xi", distanceX);
    SmartDashboard.putNumber("Distance Yi", distanceY);
    this.swerve = swerve;
    this.controller = swerve.getSwerveController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetOdometry();
    swerve.zeroGyro();
    lastTimestamp = Timer.getFPGATimestamp();
    lastErrorX = 0;
    lastErrorY = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speedX = 0;
    double speedY = 0;
    
    
    finish = true;
    if(Math.abs(swerve.getPose().getX())<Math.abs(distanceX))
    {
      finish = false;
    }
    if(Math.abs(swerve.getPose().getY())<Math.abs(distanceY))
    {
      finish = false;
    }

    // Cálculos -PID-
    double sensorX = swerve.getPose().getX();
    double errorX = distanceX - sensorX;
    speedX = Auton.kp*errorX;

    double sensorY = swerve.getPose().getY();
    double errorY = distanceY - sensorY;
    speedY = Auton.kp*errorY;

  
    
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorRateX = (errorX - lastErrorX) / dt;
    double errorRateY = (errorY - lastErrorY) / dt;

    errorSumX += errorX * dt;
    errorSumY += errorY * dt;

    speedX = Auton.kp * errorX + Auton.ki * errorSumX + Auton.kd * errorRateX;
    speedY = Auton.kp * errorY + Auton.ki * errorSumY + Auton.kd * errorRateY;
    lastTimestamp = Timer.getFPGATimestamp();
    lastErrorX = errorX;
    lastErrorY = errorY;

    double xVelocity   = Math.pow(speedX, 3);
    double yVelocity   = Math.pow(speedY, 3);
    //velocidade do giro
    double angVelocity = Math.pow(0, 3);
    
  
    // Drive using raw values.
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * controller.config.maxAngularVelocity,
                 true ,false);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}