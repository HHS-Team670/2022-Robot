package frc.team670.robot.subsystems;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

/**
 * Connects the intake to the shooter
 * 
 * @author Armaan, Soham, Edward
 */
public class ConveyorSystem extends MustangSubsystemBase {

    SparkMAXLite conveyor1Motor;
    SparkMAXLite conveyor2Motor;
    BeamBreak bb1;
    BeamBreak bb2;
    int balls;

  

    public ConveyorSystem() {
        conveyor1Motor = SparkMAXFactory.buildSparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, SparkMAXFactory.defaultConfig,
                Motor_Type.NEO_550);
        conveyor2Motor = SparkMAXFactory.buildSparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR,SparkMAXFactory.defaultConfig,
                Motor_Type.NEO_550);
        bb1 = new BeamBreak(RobotMap.INTAKE_CONVEYOR_BEAMBREAK);

        bb2 = new BeamBreak(RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);
    }
    
      public void setIntakeConveyor(double speed) {
       
        conveyor1Motor.set(speed);
        // conveyor1Motor.set(0.7);
      }
     
      public void setShooterConveyor(double speed) {
        conveyor2Motor.set(speed);
      }
      //conveyorBalls=conveyor.numOfBalls() 
      // assuming there are 2 balls
      // conveyorBalls will equal 2 
      public int numOfBall() {
            if(bb1.isTriggered()) {
             balls = 1;
            if(bb2.isTriggered())
             balls = 1;
            if(bb1.isTriggered() && bb2.isTriggered())
             balls = 2;
            if(( !bb1.isTriggered()&&!bb2.isTriggered() ))
             balls = 0;
            }
            return balls;
      }


    @Override
    public HealthState checkHealth() {
        
		return HealthState.GREEN;

    }

    public void setConveyorClosed() {
        //   if(bb1.isTriggered());
       //   double motorSpeed = 0;
       //conveyor1Motor.set(motorSpeed);

           if(bb2.isTriggered()){
               conveyor2Motor.set(0);
               if(bb1.isTriggered())
               conveyor1Motor.set(0);
           }
         


        }
    @Override
    public void mustangPeriodic() {
        setConveyorClosed();
    }

    @Override
    public void debugSubsystem() {
        // intakeConveyor.debugBeamBreaks();
		// shooterConveyor.debugBeamBreaks();
    }

    public boolean isRunning() {
        if( !(conveyor1Motor.get() == 0) || !(conveyor2Motor.get() == 0))
        return true;
        else
        return false;
         
    }

    public void turnConveyorOff() {
        conveyor1Motor.set(0);
        conveyor2Motor.set(0);
    } 

}

    