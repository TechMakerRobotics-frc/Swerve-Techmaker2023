
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevadorConstants;


public class Elevador extends SubsystemBase {

    private static Elevador instance;
  boolean extended = false;

  //Dois motores, um de  cada lado 
  CANSparkMax  motorLeft = new CANSparkMax(ElevadorConstants.kElevadorLeftMotor,MotorType.kBrushless);
  CANSparkMax  motorRight = new CANSparkMax (ElevadorConstants.kElevadorRightMotor,MotorType.kBrushless);
  
  //dois encoders, um de cada motor
  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  // Elevador ta ai
  public Elevador() {
    
    //Limpa qualquer configuração  inicial dos modulos
    motorLeft.restoreFactoryDefaults();
    motorRight.restoreFactoryDefaults();

    //Configura para  que o  motor se mantenha estatico quando em 0
    motorLeft.setIdleMode(IdleMode.kCoast);
    motorRight.setIdleMode(IdleMode.kCoast);
    //Configura a rampa de aceleração para evitar picos de corrente elétrica
    motorLeft.setOpenLoopRampRate(ElevadorConstants.kRampRate);
    motorRight.setOpenLoopRampRate(ElevadorConstants.kRampRate);

    //Inverto o motor da esquerda para que girem juntos
    motorLeft.setInverted(true);

    //Associo os encoders, seto a razão de 1 volta e zero os mesmos
    leftEncoder = motorLeft.getEncoder();
    rightEncoder = motorRight.getEncoder();
  
    leftEncoder.setPositionConversionFactor(1);
    rightEncoder.setPositionConversionFactor(1);
    resetEncoder();
    
  }
  public static Elevador getInstance() {
    if (instance == null) {
        instance = new Elevador();
    }
    return instance;
}
  //Função principal que movimenta o braço para frente(+) e  para tras(-)
  public void setMotorPower(double power) {
    SmartDashboard.putNumber("Elevador Potencia (%)", power * 100.0);
      motorRight.set(power);
      motorLeft.set(power);
       
    
  }

  //Reseta os valores dos encoders, para ter a referencia atual
  public void resetEncoder(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  
  //Função  que captura  os encoders, fazendo uma media dos dois lados e dividindo pela redução
  public double getEncoder(){
    return (((rightEncoder.getPosition()+leftEncoder.getPosition())/2)*ElevadorConstants.kGearRatio);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevador Encoder", getEncoder());
   
  }


}