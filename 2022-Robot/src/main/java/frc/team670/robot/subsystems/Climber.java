package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class Climber extends MustangSubsystemBase {

    public enum Level {
        LOW, MID, HIGH, INTERMEDIATE_HIGH
    }

    private double kFF;
    private double kP;

    private final double MAX_ACC;
    private final double MIN_VEL;
    private final double MAX_VEL;

    private SparkMaxPIDController leadController;
    private RelativeEncoder leadEncoder;

    private boolean onBar;
    private double target;

    private int currentAtHookedCount;

    private final double MOTOR_ROTATIONS_AT_RETRACTED;
    private final double MOTOR_ROTATIONS_AT_MAX_EXTENSION;

    private final double SOFT_LIMIT_AT_RETRACTED;
    private final double SOFT_LIMIT_AT_EXTENSION;

    private final double ROTATIONS_PER_CM;
    private final double ROTATIONS_HALF_CM;

    private final double MOTOR_ID;

    private SparkMAXLite motor;
    private SparkMaxLimitSwitch limitSwitch;

    private boolean isReversed;

    /**
     * @param motorId The CAN id for the motor controller
     * @param ff The Feed Forward value
     * @param p The P value from PID
     * @param isReversed Whether the climber direction is flipped (negative is outwards, positive is inwards)
     * @param motorRotationsAtRetracted Number of rotations when climber is all the way in
     * @param motorRotationsAtMaxExtension Number of rotations when climber is all the way out
     * @param rotationsPerCM 
     * @param maxAcc Max acceleration the climber can take
     * @param maxVel Max velocity the climber can take
     * @param minVel Minimum velocity the climber can take
     */
    public Climber(int motorId, double ff, double p, boolean isReversed, double motorRotationsAtRetracted,
            double motorRotationsAtMaxExtension, double rotationsPerCM, double maxAcc, double maxVel, double minVel) {
        
                this.kFF = ff;
        this.kP = p;
        this.MOTOR_ROTATIONS_AT_RETRACTED = motorRotationsAtRetracted;
        this.MOTOR_ROTATIONS_AT_MAX_EXTENSION = motorRotationsAtMaxExtension;
        this.SOFT_LIMIT_AT_RETRACTED = this.MOTOR_ROTATIONS_AT_RETRACTED + .5f;
        this.SOFT_LIMIT_AT_EXTENSION = MOTOR_ROTATIONS_AT_MAX_EXTENSION - 1;
        this.MAX_ACC = maxAcc;
        this.MAX_VEL = maxVel;
        this.MIN_VEL = minVel;
        this.MOTOR_ID = motorId;
        this.isReversed = isReversed;

        ROTATIONS_PER_CM = rotationsPerCM;
        ROTATIONS_HALF_CM = 0.5 * ROTATIONS_PER_CM;

        motor = SparkMAXFactory.buildFactorySparkMAX(motorId, Motor_Type.NEO);
        motor.setIdleMode(IdleMode.kBrake);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) SOFT_LIMIT_AT_EXTENSION);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float) SOFT_LIMIT_AT_RETRACTED);

        if (!isReversed) {
            limitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        } else {
            limitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

        }
        limitSwitch.enableLimitSwitch(true);

        leadController = motor.getPIDController();
        leadEncoder = motor.getEncoder();
        leadEncoder.setPosition(0);
        setSmartMotionConstants();

        onBar = false;
        target = 0.0;
        currentAtHookedCount = 0;
        SmartDashboard.putNumber("Climber Target on " + MOTOR_ID + ":", target);
    }

    public void setSmartMotionConstants() {
        leadController.setFF(kFF);
        leadController.setP(kP);
        leadController.setSmartMotionMaxVelocity(MAX_VEL, ClimberContainer.SMARTMOTION_SLOT);
        leadController.setSmartMotionMinOutputVelocity(MIN_VEL, ClimberContainer.SMARTMOTION_SLOT);
        leadController.setSmartMotionMaxAccel(MAX_ACC, ClimberContainer.SMARTMOTION_SLOT);
        leadController.setSmartMotionAllowedClosedLoopError(ClimberContainer.ALLOWED_ERR_ROTATIONS,
                ClimberContainer.SMARTMOTION_SLOT);
    }

    public void hookOnBar() {
        if (isHooked() && !onBar) {
            onBar = true;
        }

        if (!onBar) {
            run(-ClimberContainer.HOOKING_POWER);
        }
    }

    public void run(double power) {
        if(isReversed)
            power *= -1;
        motor.set(power);
    }

    public void unhookFromBar() {
        if (onBar) {
            motor.set(ClimberContainer.HOOKING_POWER);
            onBar = false;
        }
    }

    private boolean isHooked() {
        double current = getMotorCurrent();
        if (current >= ClimberContainer.NORMAL_OUTPUT) {
            currentAtHookedCount++;
        } else {
            currentAtHookedCount = 0;
        }
        if (currentAtHookedCount >= 4) { // 4 consecutive readings higher than peak
            return true;
        }
        return false;

    }

    public boolean isHookedOnBar() {
        return onBar;
    }

    public void climb(double rotations) {
        target = rotations;
        Logger.consoleLog("Climber with Motor ID %s rotation target %s", MOTOR_ID, rotations);
        leadController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion);
    }

    // is called when retracting c1 just enough so c2 can extend in the right
    // position
    // doesn't retract completely
    public void retract() {
        climb(MOTOR_ROTATIONS_AT_RETRACTED);
    }

    public void climbToHeight(Level level) {
        switch (level) {
            case LOW:
                climb(ClimberContainer.LOW_BAR_TARGET_HEIGHT_CM * ROTATIONS_PER_CM
                        - ClimberContainer.VERTICAL_STARTING_HEIGHT_CM * ROTATIONS_PER_CM);
            case MID:
                climb(ClimberContainer.MID_BAR_TARGET_HEIGHT_CM * ROTATIONS_PER_CM
                        - ClimberContainer.VERTICAL_STARTING_HEIGHT_CM * ROTATIONS_PER_CM);
            case INTERMEDIATE_HIGH:
                climb(ClimberContainer.MOTOR_ROTATIONS_TO_PARTIAL_EXTENSION);
            case HIGH:
                climb(MOTOR_ROTATIONS_AT_MAX_EXTENSION);
        }

    }

    public HealthState checkHealth() {
        if ((motor == null || motor.getLastError() != REVLibError.kOk)) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    public boolean isAtTarget() {
        return (Math.abs(leadEncoder.getPosition() - target) < ROTATIONS_HALF_CM);
    }

    public boolean reverseLimitSwitchTripped() {
        return limitSwitch.isPressed();
    }

    protected double getUnadjustedMotorRotations() {
        return this.leadEncoder.getPosition();
    }

    protected double getMotorCurrent() {
        return this.motor.getOutputCurrent();
    }

    public void test() {
        motor.set(SmartDashboard.getNumber("Climber power", 0.0));
        SmartDashboard.putNumber("Climber motor rotations", getUnadjustedMotorRotations());
    }

    public void stop() {
        motor.set(0);
    }

    @Override
    public void mustangPeriodic() {
        if (reverseLimitSwitchTripped()) {
            leadEncoder.setPosition(0);
        }
    }

    @Override
    public void debugSubsystem() {
        double rotations = SmartDashboard.getNumber("Climber Target on " + MOTOR_ID + ":", 0);
        climb(rotations);
    }

}