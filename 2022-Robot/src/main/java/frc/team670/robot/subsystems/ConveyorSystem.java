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

    SparkMAXLite intakeMotor;
    SparkMAXLite shooterMotor;

    BeamBreak intakeBeamBreak;
    BeamBreak shooterBeamBreak;

    public ConveyorSystem() {
        intakeMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        shooterMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR, Motor_Type.NEO_550);
        intakeBeamBreak = new BeamBreak(RobotMap.INTAKE_CONVEYOR_BEAMBREAK);
        shooterBeamBreak = new BeamBreak(RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);

        //Independantly control each motor
        //Prevent balls from accidentally shooting
        //when the shooter beam break is triggered we want the shooter conveyor to stop
        //when the intake beam break is triggered and the shooter conveyor is full, we want the intake conveyor to stop
        //methods: stopAll(), isRunning()
    }

    @Override
    public HealthState checkHealth() {
        return null;
    }

    @Override
    public void mustangPeriodic() {
        if (shooterBeamBreak.isTriggered()) {
            shooterMotor.set(0);
        }
    }

    public void stopAll() {
        intakeMotor.set(0);
        shooterMotor.set(0);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }

    @Override
    public void debugSubsystem() {
        
    }
}
