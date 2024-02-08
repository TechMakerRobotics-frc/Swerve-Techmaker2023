package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;
  boolean extended = false;
  //Dois motores, um de  cada lado 
  CANSparkMax  motorLeft = new CANSparkMax(ElevatorConstants.kElevatorLeftMotor,MotorType.kBrushless);
  CANSparkMax  motorRight = new CANSparkMax (ElevatorConstants.kElevatorRightMotor,MotorType.kBrushless);
  
  //dois encoders, um de cada motor
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;
  /** Creates a new arm. */
  public ElevatorSubsystem() {
    
    //Limpo qualquer configuração  inicial dos modulos
    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();

    //Configuro para  que o  motor se mantenha estatico quando em 0
    motorLeft.setIdleMode(IdleMode.kCoast);
    motorRight.setIdleMode(IdleMode.kCoast);
    
    //Inverto o motor da esquerda para que girem juntos
    motorLeft.setInverted(true);

    //Associo os encoders, seto a razão de 1 volta e zero os mesmos
    leftEncoder = motorLeft.getEncoder();
    rightEncoder = motorRight.getEncoder();
  
    resetEncoder();
    
  }
  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
        instance = new ElevatorSubsystem();
    }
    return instance;
}
  //Função principal que movimenta o braço para frente(+) e  para tras(-)
  public void setMotorPower(double Up) {
    SmartDashboard.putNumber("Elevator Potencia (%)", Up * 100.0);
      motorRight.set(Up);
      motorLeft.set(Up);
    



      
    
    
  }

  //Reseta os valores dos encoders, para ter a referencia atual
  public void resetEncoder(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  
  //Função  que captura  os encoders, fazendo uma media dos dois lados e dividindo pela redução
  public double getEncoder(){
    return (((rightEncoder.getPosition()+leftEncoder.getPosition())/2)*ElevatorConstants.kGearRatio);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder", getEncoder());
   
  }


}