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
 * @author Kedar
 */
public class ConveyorSystem extends MustangSubsystemBase {

    private SparkMAXLite intakeMotor;
    private SparkMAXLite shooterMotor;

    private BeamBreak intakeBeamBreak;
    private BeamBreak shooterBeamBreak;


    public ConveyorSystem(){
        intakeMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        shooterMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR, Motor_Type.NEO_550);
    
        intakeBeamBreak = new BeamBreak(RobotMap.INTAKE_CONVEYOR_BEAMBREAK);
        shooterBeamBreak = new BeamBreak(RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);
    
    }
    
    public int BallTracker(){
        int count = 0;

        if(intakeBeamBreak.isTriggered()){
            count++;
        }
        if(shooterBeamBreak.isTriggered()){
            count++;
        }
        return count;
        
    }

    public void intakeSpeed(double speed){
        if(speed >= 0 && speed <=1 ){
            intakeMotor.set(speed);
        }
    }

    public void shooterSpeed(double speed){
        if(speed >= 0 && speed <= 1){
            shooterMotor.set(speed);
        }
    }

    public void Pileup(){
        boolean shooterTrigger = false;
        
        if(shooterBeamBreak.isTriggered()){
            shooterMotor.stopMotor();
            shooterTrigger = true;
        }
        
        if(intakeBeamBreak.isTriggered()){
            if(shooterTrigger){
                intakeMotor.stopMotor();
            } 
        }

    }

    public boolean isOn(){
        if(intakeMotor.get() != 0 && shooterMotor.get() != 0){
            return true;
        } else{
            return false;
        }
    }

    public void Stop(){
        shooterMotor.stopMotor();
        intakeMotor.stopMotor();
    }

    
    

    @Override
    public HealthState checkHealth() {
        
        // TODO Auto-generated method stub
        return null;
    }


    @Override
    public void mustangPeriodic() {
        Pileup();
      
        
    }


    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Balls", BallTracker());
        
        // TODO Auto-generated method stub
        
    }

    public void setConveyorMode(ConveyorSystem conveyor) {
    }
}
