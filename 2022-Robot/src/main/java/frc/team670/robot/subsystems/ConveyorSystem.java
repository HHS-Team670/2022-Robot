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

    SparkMAXLite intakeConveyorMotor;
    SparkMAXLite shooterConveyorMotor;

    BeamBreak intakeConveyorBeamBreak;
    BeamBreak shooterConveyorBeamBreak;

    public ConveyorSystem() {
        intakeConveyorMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        shooterConveyorMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        intakeConveyorBeamBreak = new BeamBreak(RobotMap.INTAKE_CONVEYOR_BEAMBREAK);
        shooterConveyorBeamBreak = new BeamBreak(RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);
    }

    @Override
    public HealthState checkHealth() {
        return null;
    }

    @Override
    public void mustangPeriodic() {
        
    }

    @Override
    public void debugSubsystem() {
        
    }
}
