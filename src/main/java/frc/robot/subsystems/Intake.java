
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private static Intake instance;
  
  // Motor ta ai
  CANSparkMax  motor = new CANSparkMax(IntakeConstants.kIntakeMotor,MotorType.kBrushless);
  
  // Intake ta ai tambem
  public Intake() {

    //Configuro para  que o  motor se mantenha estatico quando em 0
    motor.setIdleMode(IdleMode.kCoast);
    
    //Configuro a rampa de aceleração para evitar picos de corrente elétrica
    motor.setOpenLoopRampRate(IntakeConstants.kRampRate);

    //Inverto o motor para ele girar de forma correta.
    motor.setInverted(false);
    
    
  }
  public static Intake getInstance() {
    if (instance == null) {
        instance = new Intake();
    }
    return instance;
}


public void setMotorPower(double forward) {
  SmartDashboard.putNumber("Intake Potencia (%)", forward * 100.0);
    motor.set(forward);
}

 


}