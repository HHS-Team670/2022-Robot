package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;

/*
 * Governs the intake subsystem
 * @author Khicken, Sanatan, Armaan, Soham
*/
public class Intake extends MustangSubsystemBase {

    private static final double INTAKE_ROLLER_SPEED = 0.84; // Experimentally found

	private static final int JAMMED_COUNT_DEF = 150;

    private double INTAKE_PEAK_CURRENT = 35; // Testing
    private static final int PID_SLOT = 0; // Change later

    private int exceededCurrentLimitCount = 0;

    private SparkMAXLite roller;
    private SparkMAXLite deployerMotor;
    public Deployer deployer;
    private boolean isDeployed = false; // TODO: true for testing, change this

    public int countWasJammed = 0;

    public Intake() {
        // Intake roller should be inverted
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
        deployerMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_DEPLOYER, Motor_Type.NEO_550);
        deployer = new Deployer(deployerMotor, PID_SLOT);
        roller.setInverted(true);
    }

    // Returns true if the intake is rolling
    public boolean isRolling() {
        return (roller.get() != 0);
    }

    // Returns true if the intake is deployed
    public boolean isDeployed() {
        return deployer.isDeployed();
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

    public int unjam(boolean reversed) {
        if (isJammed()) {
            countWasJammed = Intake.JAMMED_COUNT_DEF;
        }
        if (countWasJammed > 0) {
            roll(!reversed);
            countWasJammed--;
        } else {
            roll(reversed);
        }
        return countWasJammed;
    }

    // Stops the intake
    public void stop() {
        roller.stopMotor();
        Logger.consoleLog("Intake stopped");
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
        if (deployerMotor == null) {
            if (isDeployed) {

                return HealthState.YELLOW;
            }

            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        boolean reversed = false;
        do {
            unjam(reversed);
            reversed = !reversed;
        } while (isJammed());
    }

}