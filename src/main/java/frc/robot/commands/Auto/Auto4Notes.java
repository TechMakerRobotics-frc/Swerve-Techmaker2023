// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SetIntake;
import frc.robot.commands.ResetIntake;
//import frc.robot.commands.Shoot;
import frc.robot.commands.swervedrive.auto.MoveXYHeading;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto4Notes extends SequentialCommandGroup {

  public Auto4Notes(SwerveSubsystem drivebase) {

    addCommands(
      new MoveXYHeading(-2.1, 0, 0, drivebase),
      new SetIntake(),
      new WaitCommand(1),
      new ResetIntake(),
      new MoveXYHeading(2.1, 0, 0, drivebase)
      //new Shoot()
      );
  }
}