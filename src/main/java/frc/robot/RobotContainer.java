
package frc.robot;

import java.io.File;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ResetShoot;
import frc.robot.commands.SetShooter;
import frc.robot.commands.Auto.Auto4Notes;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer
{
   private final SwerveSubsystem drivebase;
   private final IntakeSubsystem intake  = IntakeSubsystem.getInstance();
   //private final PhotonVision photonVision = new PhotonVision();
   // Subtitua por CommandPS4Controller ou CommandJoystick se necessário.
   CommandXboxController driverXbox = new CommandXboxController(2);
   CommandXboxController driverXboxOperator = new CommandXboxController(0);

    
  TeleopDrive closedFieldRel;


    public RobotContainer(){
      drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
      closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                  OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                  OperatorConstants.LEFT_X_DEADBAND),
        () -> (driverXbox.getRawAxis(3)-driverXbox.getRawAxis(2)), () -> true);

    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     //* CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    // Configura os botões do Xbox.
    void configureBindings(){
        driverXbox.povRight().onTrue(new InstantCommand(drivebase::zeroGyro));
        driverXbox.povLeft().onTrue(new InstantCommand(drivebase::resetOdometry));
        driverXbox.a().onTrue(new InstantCommand(drivebase::lock));
        driverXbox.y().onTrue(new Auto4Notes(drivebase));
        
        driverXboxOperator.x()
        .onTrue(new SetShooter())
        .onFalse(new ResetShoot());
        
        driverXboxOperator.y()
        .onTrue(new InstantCommand(()->intake.setMotorPower(IntakeConstants.kPower),intake))
        .onFalse(new InstantCommand(()->intake.setMotorPower(0),intake));
        
        driverXboxOperator.b()
        .onTrue(new InstantCommand(()->intake.setMotorPower(IntakeConstants.kReversePower),intake))
        .onFalse(new InstantCommand(()->intake.setMotorPower(0),intake));
        
        drivebase.setDefaultCommand(closedFieldRel);



    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return null;
  }


  public void setDriveMode(){
    //drivebase.setDefaultCommand();
  }
}