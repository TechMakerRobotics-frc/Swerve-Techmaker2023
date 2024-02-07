package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class UpElevator extends InstantCommand {

    private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    @Override
    public void initialize() {
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {
  
        elevator.setMotorPower(ElevatorConstants.kPowerUp);
    }
  }
    
