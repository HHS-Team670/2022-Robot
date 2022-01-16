package frc.team670.robot.subsystems;

import com.revrobotics.CANError;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

import frc.team670.robot.constants.RobotMap;

public class Conveyor extends MustangSubsystemBase {

    private SparkMAXLite roller;

    private double CONVEYOR_SPEED = 0.4; // % output from testing 2/16.

    public Conveyor() {
        // Conveyor motor should not be inverted
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CONVEYOR_ROLLER, Motor_Type.NEO_550);
    }

    /**
     * 
     * @param reversed True to run the conveyor in reverse
     */
    public void run(boolean reversed) {
        if (reversed) {
            roller.set(CONVEYOR_SPEED * -1);
        } else {
            roller.set(CONVEYOR_SPEED);
        }
    }

    public void stop() {
        roller.set(0);
    }

    @Override
    public HealthState checkHealth() {
        if (roller.getLastError() != null && roller.getLastError() != CANError.kOk)
            return HealthState.RED;
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

}