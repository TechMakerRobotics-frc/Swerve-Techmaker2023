package frc.robot.commands;

import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class InsideClaw extends InstantCommand {

    private final ClawSubsystem elevator = ClawSubsystem.getInstance();

    @Override
    public void initialize() {
      addRequirements(elevator);
    }
  
    @Override
    public void execute() {
  
        elevator.setMotorPower(ClawConstants.kpowerInside);
    }
  }
    
