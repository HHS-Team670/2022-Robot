package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team670.mustanglib.dataCollection.sensors.PicoColorMatcher;
import frc.team670.mustanglib.dataCollection.sensors.PicoColorSensor;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.commands.routines.intake.EjectCargo;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.ConveyorSystem.Status;

/*
 * Represents the intake roller. Does NOT control the
 * deployer (moves the intake arm down & up) or the 
 * conveyor system (rollers that bring ball from intake to outtake)
 * @author Khicken, Sanatan, Armaan, Jerry
*/
public class IntakeColorMatcher extends MustangSubsystemBase {

    private static double intake_roller_speed = 0.6; // Experimentally found

    public static final int JAMMED_COUNT_DEF = 150;

    private double INTAKE_PEAK_CURRENT = 50; // Testing

    private int exceededCurrentLimitCount = 0;

    private SparkMAXLite roller;

    private ConveyorSystem conveyor;
    private Deployer deployer;

    private int rejectCount = 0;

    private static PicoColorMatcher picoColorMatcher = new PicoColorMatcher();

    public IntakeColorMatcher(ConveyorSystem conveyor, Deployer deployer) {
        this.conveyor = conveyor;
        this.deployer = deployer;
        setName("Intake");
        setLogFileHeader("isConveyorOff", "conveyorBallCount");
        // Intake roller should be inverted

        roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
        roller.setInverted(true);
        roller.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Returns true if the roller is moving
     * 
     * @return True if the motor is rolling
     */
    public boolean isRolling() {
        return (roller.get() != 0);
    }

    /**
     * Runs the main intake motor in the specified direction
     * 
     * @param reversed True for backwards, false for forwards
     */
    public void roll(boolean reversed) {
        deployer.deploy(true);
        if (conveyor.getBallCount() == 2) {
            return;
        }
        if (reversed) {
            roller.set(intake_roller_speed * -1);

        } else {
            roller.set(intake_roller_speed);
        }
    }

    /**
     * Returns true if the intake is jammed. Detected by checking if the current
     * stays above the peak current for 4 or more cycles
     * 
     * @return
     */
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

    /**
     * Stops the intake rollers, retracts the intake, and stops all conveyor
     * rollers.
     */
    public void stop() {
        roller.stopMotor();
        if (deployer.isDeployed()) {
            deployer.deploy(false);
        }
        conveyor.stopAll();
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
        debugSubsystem();
        if (conveyor.getStatus() == ConveyorSystem.Status.OFF && conveyor.getBallCount() == 2) {
            stop();
        }

        //If rejected keep rejecting until enough time has passed to say it has been successfull rejected
        if(rejectCount > 0){
            rejectCount++;
            if (rejectCount > 4) { //TESTING need to fine tune
                rejectCount = 0; 
                conveyor.setConveyorMode(Status.INTAKING);
            }
        }
        else if(wrongColor()){
            conveyor.setConveyorMode(Status.EJECTING); //Maybe use EjectCargo class?
            rejectCount = 1;
        }
    }

    @Override
    public void debugSubsystem() {
        boolean isConveyorOff = (conveyor.getStatus() == ConveyorSystem.Status.OFF);
        int conveyorBallCount = conveyor.getBallCount();
        boolean colorSensorConnected = picoColorMatcher.isSensor0Connected();
        int color = picoColorMatcher.detectColor();
        super.writeToLogFile(isConveyorOff, conveyorBallCount);
        super.writeToLogFile(colorSensorConnected, color);
    }

    public boolean wrongColor(){
        int color = picoColorMatcher.detectColor();
        //If we know with enough certainty it is the other alliance's color, reject it
        return DriverStation.getAlliance() == Alliance.Blue ? color == PicoColorMatcher.colors.RED.getColorNumber() : color == PicoColorMatcher.colors.BLUE.getColorNumber();
    }
}