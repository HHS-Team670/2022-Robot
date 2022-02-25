package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

/*
 * Governs the intake subsystem
 * @author Khicken, Sanatan, Armaan
*/
public class Intake extends MustangSubsystemBase {

    private static double intake_roller_speed = 0.6; // Experimentally found

	public static final int JAMMED_COUNT_DEF = 150;

    private double INTAKE_PEAK_CURRENT = 50; // Testing

    private int exceededCurrentLimitCount = 0;

    private SparkMAXLite roller;

    private ConveyorSystem conveyor;
    private Deployer deployer;

    public Intake(ConveyorSystem conveyor, Deployer deployer) {
        this.conveyor = conveyor;
        this.deployer = deployer;
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
        if (!deployer.isDeployed()) {
            deployer.deploy(true);
        }
        if (reversed) {
            roller.set(intake_roller_speed * -1);

        } else {
            roller.set(intake_roller_speed);
        }
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
        SmartDashboard.putBoolean("Conveyor Off", conveyor.getStatus() == ConveyorSystem.Status.OFF);
        SmartDashboard.putNumber("Conveyor Ball Count", conveyor.getBallCount());
        if(conveyor.getStatus() == ConveyorSystem.Status.OFF && conveyor.getBallCount() == 2){
            stop();
        }
    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        
    }

    public void adjustLinearSpeedBasedOnDrivebaseSpeed(){
        double drivebaseSpeed = DriveBase.getLinearSpeed();
        intake_roller_speed = drivebaseSpeed/4; //TODO: COMPLETE MATH BASED ON GEARING
    }

}