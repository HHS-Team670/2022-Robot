package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

/*
 * Governs the intake subsystem
 * @author Khicken, Sanatan, Armaan, Soham
*/
public class Intake extends MustangSubsystemBase {

    private static double intake_roller_speed = 0.6; // Experimentally found

    private static final int JAMMED_COUNT_DEF = 150;

    private double INTAKE_PEAK_CURRENT = 50; // Testing

    private int exceededCurrentLimitCount = 0;

    private SparkMAXLite roller;

    public Deployer deployer;

    private boolean isReversed;

    private boolean isDeployed = true;

<<<<<<< HEAD
    public int countWasJammed = 0;

    private final int PIDSlot = 0;

    public Intake() {
=======
    private ConveyorSystem conveyor;

    public Intake(ConveyorSystem conveyor) {
        this.conveyor = conveyor;
>>>>>>> dev
        // Intake roller should be inverted
        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
        deployer = new Deployer(SparkMAXFactory.buildFactorySparkMAX(RobotMap.DEPLOYER_MOTOR, Motor_Type.NEO), PIDSlot);
        roller.setInverted(true);
        isReversed = false;
    }

    // Returns true if the intake is rolling
    public boolean isRolling() {
        return (roller.get() != 0);
    }

    // Runs the main intake motor in the specified direction
    public void roll(boolean reversed) {
        countWasJammed = 0;
        if (isDeployed) {
            if (reversed) {
<<<<<<< HEAD
                roller.set(INTAKE_ROLLER_SPEED * -1);
                isReversed = reversed;

            } else {
                roller.set(INTAKE_ROLLER_SPEED);
                isReversed = reversed;
=======
                roller.set(intake_roller_speed * -1);

            } else {
                roller.set(intake_roller_speed);
>>>>>>> dev
            }
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

    public int unjam(boolean reversed) {
        if (isJammed()) {
            countWasJammed = Intake.JAMMED_COUNT_DEF;
        }
        if (countWasJammed > 0) {
            roll(!reversed);
            countWasJammed--;
            isReversed = reversed;
        } else {
            roll(reversed);
        }
        return countWasJammed;
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
<<<<<<< HEAD
        unjam(isReversed);
=======
        SmartDashboard.putBoolean("Conveyor Off", conveyor.getStatus() == ConveyorSystem.Status.OFF);
        SmartDashboard.putNumber("Conveyor Ball Count", conveyor.getBallCount());
        if(conveyor.getStatus() == ConveyorSystem.Status.OFF && conveyor.getBallCount() == 2){
            stop();
        }
    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        
>>>>>>> dev
    }

    public void adjustLinearSpeedBasedOnDrivebaseSpeed(){
        double drivebaseSpeed = DriveBase.getLinearSpeed();
        intake_roller_speed = drivebaseSpeed/4; //TODO: COMPLETE MATH BASED ON GEARING
    }

}