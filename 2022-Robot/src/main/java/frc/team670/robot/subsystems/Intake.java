package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

/*
 * Governs the intake subsystem
 * @author Khicken, Sanatan, Armaan
*/
public class Intake extends MustangSubsystemBase {

    private static final double INTAKE_ROLLER_SPEED = 0.6; // Experimentally found

	public static final int JAMMED_COUNT_DEF = 150;

    private double INTAKE_PEAK_CURRENT = 50; // Testing

    private int exceededCurrentLimitCount = 0;

    private SparkMAXLite roller;

    private boolean isDeployed = true;

    public Intake() {
        // Intake roller should be inverted
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
        roller.setInverted(true);
    }

    // Returns true if the intake is rolling
    public boolean isRolling() {
        return (roller.get() != 0);
    }

    // Runs the main intake motor in the specified direction
    public void roll(boolean reversed) {
        if (isDeployed) {
            if (reversed) {
                roller.set(INTAKE_ROLLER_SPEED * -1);

            } else {
                roller.set(INTAKE_ROLLER_SPEED);
            }
        }
        Logger.consoleLog("Running intake at: %s", roller.get());
    }

    // Returns true if the intake is jammed
    public boolean isJammed() {
        double intakeCurrent = roller.getOutputCurrent();
        if (intakeCurrent > 0.2) {
            if (intakeCurrent >= INTAKE_PEAK_CURRENT) {
                exceededCurrentLimitCount++;
            } else {
                exceededCurrentLimitCount = 0;
            }
            if (exceededCurrentLimitCount >= 4) { // 4 consecutive readings higher than peak
                return true;
            }
        }
        return false;
    }

    // Stops the intake
    public void stop() {
        roller.stopMotor();
    }

    /**
     * @return RED if the roller has issues, or the intake isn't deployed but the
     *         intake motors have issues
     */

    @Override
    public HealthState checkHealth() {
        if (roller == null || roller.isErrored()) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        if(ConveyorSystem.getStatus() != ConveyorSystem.Status.OUTTAKING && ConveyorSystem.getBallCount() == 2){
            stop();
        }
    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        
    }

}