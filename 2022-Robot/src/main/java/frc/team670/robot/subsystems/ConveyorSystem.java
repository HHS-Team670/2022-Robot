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
    int fix;
    boolean fixerBB1 = false;
    boolean fixerBB2 = false;

    public ConveyorSystem() {
        conveyor1Motor = SparkMAXFactory.buildSparkMAX(RobotMap.INTAKE_CONVEYOR_MOTOR, SparkMAXFactory.defaultConfig,
                Motor_Type.NEO_550);
        conveyor2Motor = SparkMAXFactory.buildSparkMAX(RobotMap.SHOOTER_CONVEYOR_MOTOR, SparkMAXFactory.defaultConfig,
                Motor_Type.NEO_550);
        bb1 = new BeamBreak(RobotMap.INTAKE_CONVEYOR_BEAMBREAK);
        bb2 = new BeamBreak(RobotMap.SHOOTER_CONVEYOR_BEAMBREAK);
    }

    public void setC1(double speed) {
        conveyor1Motor.set(speed);
    }

    public void setC2(double speed) {
        conveyor2Motor.set(speed);
    }

    // What if the if
    // statement is triggered multiple times,
    // the ball count would exceed 2. You may want to just
    // compute the ball count inside a method and not store it
    // as a field
    /*
     * public void ballTracker() {
     * if (bb2.isTriggered() == true && fixerBB2 == false) {
     * balls ++;
     * fixerBB2 = true;
     * conveyor2Motor.stopMotor();
     * }
     * if (bb1.isTriggered() == true && fixerBB1 == false) {
     * fixerBB1 = true;
     * if (bb2.isTriggered()) {
     * conveyor1Motor.stopMotor();
     * }
     * }
     * if (bb2.isTriggered() == false) {
     * fixerBB2 = false;
     * }
     * if (!bb1.isTriggered()) {
     * fixerBB1 = false;
     * }
     * if (fixerBB1 == false && fixerBB2 == false) {
     * setC1(1);
     * setC2(1);
     * }
     * }
     */

    public int getBallCount() {
        balls = 0;
        if (bb2.isTriggered() || bb1.isTriggered()) {
            balls++;
        }
        if (bb1.isTriggered() && bb2.isTriggered()) {
            balls++;
        }
        return balls;
    }

    public void stopConveyor2() {
        if (getBallCount() == 1 && bb2.isTriggered()) {
            conveyor2Motor.stopMotor();
        }
    }
    public void stopConveyor1() {
        if (getBallCount() == 2 && bb1.isTriggered()) {
            conveyor1Motor.stopMotor();
        }
    }


    public void stopAll() {
        conveyor2Motor.stopMotor();
        conveyor1Motor.stopMotor();
    }

    // Add a get ball count method
    // Add independant stop methods
    //

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

        getBallCount();
        stopConveyor2();
    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("Balls", balls);
    }

}
