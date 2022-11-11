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
    SparkMAXLite conveyorMotor1;
    SparkMAXLite conveyorMotor2;
    BeamBreak beam1;
    BeamBreak beam2;
    int ballCount = 0;
    
    //Declare vairables, conveyors and beambreaks.
    public ConveyorSystem(){
        conveyorMotor1 = SparkMAXFactory.buildSparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
        conveyorMotor2 = SparkMAXFactory.buildSparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
        beam1= new BeamBreak(RobotMap.INTAKE_CONVEYOR_BEAMBREAK);
        beam2= new BeamBreak(RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);
        

        
    }

    
        //Set speed of motor 1
    public void setC1(double speed){
        conveyorMotor1.set(speed);
    }
        
    //Set speed of motor 2
    public void setC2(double speed){
        conveyorMotor2.set(speed);
    }


    @Override
    public HealthState checkHealth() {
        return null;
    }


    @Override
    public void mustangPeriodic() {
        if (beam2.isTriggered() == true && ballCount == 0){
            ballCount++;
            setC2(0);
        } 
        if (beam1.isTriggered() == true && ballCount == 1){
            setC1(0);
            ballCount++;
        } 
        if (ballCount == 2){
            setC1(0);
            setC2(0);
        }
    }


    @Override
    public void debugSubsystem() {
        
    }


    

}

