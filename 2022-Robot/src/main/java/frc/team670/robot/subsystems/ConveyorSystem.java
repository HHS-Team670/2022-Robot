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
    SparkMAXLite Conveyor1Motor;
    SparkMAXLite Conveyor2Motor;
    BeamBreak bb1;
    BeamBreak bb2;
    public Conveyor(){
        Conveyor1Motor=SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        Conveyor2Motor=SparkMAXFactory.buildFactorySparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        bb1=new BeamBreak(RobotMap.INTAKE_CONVEYOR_BEAMBREAK);
        bb2=new BeamBreak(RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);
        
        
    }
    public void SetC1(double speed) {
        Conveyor1Motor.set(speed);



    }
    public void SetC2(double speed) {
        Conveyor2Motor.set(speed);


    }
    @Override
    public HealthState checkHeath(){
        return null;

    }
    @Override
    public void mustangPeriodic(){
        bb1.IsTriggered;


    }
}
