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
    boolean shooterTrigger = false;
    int status = 0;


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

    public void Intake(){
        
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

    //Shoot
    public void Shoot(){
        if(shooterTrigger && intakeBeamBreak.isTriggered()){
            shooterMotor.set(0.7);
            intakeMotor.set(0.7);
            Intake();
        }
    }

    //Eject
    public void Eject(){
        shooterMotor.set(-0.7);
        intakeMotor.set(-0.7);
    }

    //Setting the status
    public void setStatus(int x){
        if(x == 1){
            Intake();
            status = 1;
        } else if(x == 2){
            Shoot();
            status = 2;
        } else if(x == 3){
            Eject();
            status = 3;
        }
    }
    //get status
    public int getStatus(){
        return status;
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
        Intake();
      
        
    }


    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Balls", BallTracker());
        
        // TODO Auto-generated method stub
        
    }
}
