
package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import swervelib.math.Matter;
 
public final class Constants
{

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final class Auton
    {
        public static final double kp = 0.75;
        public static final double ki = 0.38;
        public static final double kd = 0;
        
        // P = 0.46 - I = 0.17 - D = 0.01 

        public static final double kpH = 0;
        public static final double kiH = 0;
        public static final double kdH = 0;

        // P = 0 - I = 0 - D = 0 


       /* velForward = drivePID.calculate(limelight.getALimelight(), strafeOffSet);
        velStrafe = strafePID.calculate(limelight.getXLimelight(), strafeOffSet);
        velGiro = rotationPID.calculate(limelight.getYaw(),rotationOffset); */


        public static final double MAX_SPEED        = 4;
        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Drivebase
    {
public static final double WHEEL_LOCK_TIME = 10; // Segundos
    }

    public static class OperatorConstants
    {

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;

        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
    public static class IntakeConstants {
        public static final int kIntakeMotor = 14;
        public static final int kReturnTicks = 0;
        public static final double kRampRate = 0;
        public static final double kPower = 1;
        public static final double kReversePower = -0.3;
        public static final double kPowerWait = 0;

    }
/* 
    public static class ElevadorConstants {
        public static final int kElevadorRightMotor = 0;
        public static final int kElevadorLeftMotor = 0;
        public static final double kPower = 0;   
        public static final double kGearRatio = 0;
        public static final double kRampRate = 0; 
        public static final double kp = 0;
        public static final double ki = 0.0;
        public static final double kd = 0.01;


        }*/

        public static class ShooterConstants {
            public static final int kShooterDownMotor = 13;
            public static final int kShooterUpMotor = 15;
            public static final double kPowerDown = 0.1;
            public static final double kPower = 1;
        
          } 
          
          
    public static class PDPConstants {
        public static final int kID = 16;
        public static final ModuleType kModule = ModuleType.kRev;
        public static final double kMinimumVoltage = 10.5;
      } 
      
}
