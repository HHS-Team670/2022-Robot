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
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class Climber extends MustangSubsystemBase { // og telescoping

    private double kFF;

    private static final double HOOKING_POWER = 0.05; // TODO find this, it's the power used when hooking climber

    private final double MAX_ACC; 
    private final double MIN_VEL; 
    private final double MAX_VEL;


    private static final double ALLOWED_ERR_ROTATIONS = 3;


    private static final double MIN_RUNNING_OUTPUT_CURRENT = 0.2; // TODO find this

    // TODO find this
    private static final double NORMAL_OUTPUT = 6.5; // this should be the current output when running normally

    private int SMARTMOTION_SLOT = 0;

    private SparkMaxPIDController leadController;
    private RelativeEncoder leadEncoder;

    private boolean onBar;
    private double target;

    private int currentAtHookedCount;

    private final double MOTOR_ROTATIONS_AT_RETRACTED;
    private final double MOTOR_ROTATIONS_AT_MAX_EXTENSION;

    private final double LOW_BAR_HEIGHT_CM = 124;
    private final double MID_BAR_HEIGHT_CM = 153;

    private final double EXTENSION_DIST_ABOVE_BAR_CM = 15.24; // half a foot

    private final double LOW_BAR_TARGET_HEIGHT_CM = LOW_BAR_HEIGHT_CM + EXTENSION_DIST_ABOVE_BAR_CM;
    private final double MID_BAR_TARGET_HEIGHT_CM = MID_BAR_HEIGHT_CM + EXTENSION_DIST_ABOVE_BAR_CM;

    private final double SOFT_LIMIT_AT_RETRACTED;
    private final double SOFT_LIMIT_AT_EXTENSION;

    private final double ROTATIONS_PER_CM;
    private final double ROTATIONS_HALF_CM;

    private SparkMAXLite motor;
    private SparkMaxLimitSwitch limitSwitch;

    public Climber(int motorId, double ff, double motorRotationsAtRetracted,
            double motorRotationsAtMaxExtension, double rotationsPerCM, double maxAcc, double maxVel, double minVel) {
        kFF = ff;
        this.MOTOR_ROTATIONS_AT_RETRACTED = motorRotationsAtRetracted;
        this.MOTOR_ROTATIONS_AT_MAX_EXTENSION = motorRotationsAtMaxExtension;
        this.SOFT_LIMIT_AT_RETRACTED = this.MOTOR_ROTATIONS_AT_RETRACTED + .5f;
        this.SOFT_LIMIT_AT_EXTENSION = MOTOR_ROTATIONS_AT_MAX_EXTENSION - 10;
        this.MAX_ACC = maxAcc;
        this.MAX_VEL = maxVel;
        this.MIN_VEL = minVel;

        ROTATIONS_PER_CM = rotationsPerCM;
        ROTATIONS_HALF_CM = 0.5 * ROTATIONS_PER_CM;

        motor = SparkMAXFactory.buildFactorySparkMAX(motorId, Motor_Type.NEO);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float)SOFT_LIMIT_AT_EXTENSION);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float)SOFT_LIMIT_AT_RETRACTED);

        // TODO kNormallyOpen or kNormallyClosed
        // TODO reverse switch or forward?
        limitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen); 

        leadController = motor.getPIDController();
        leadEncoder = motor.getEncoder();
        leadEncoder.setPosition(this.MOTOR_ROTATIONS_AT_RETRACTED);
        setSmartMotionConstants();

        onBar = false;
        target = 0.0;
        currentAtHookedCount = 0;
        SmartDashboard.putNumber("Climber power", 0.0);
        SmartDashboard.putBoolean("Climber deploy", false);
        SmartDashboard.putNumber("Climber motor rotations", 0);
        SmartDashboard.putNumber("Climber rotation target", 0);
    }

    public void setSmartMotionConstants() {
        leadController.setFF(kFF);
        leadController.setSmartMotionMaxVelocity(MAX_VEL, this.SMARTMOTION_SLOT);
        leadController.setSmartMotionMinOutputVelocity(MIN_VEL, this.SMARTMOTION_SLOT);
        leadController.setSmartMotionMaxAccel(MAX_ACC, this.SMARTMOTION_SLOT);
        leadController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR_ROTATIONS, this.SMARTMOTION_SLOT);
    }

    public void hookOnBar() {
        if (isHooked() && !onBar) {
            onBar = true;
        }

        if (!onBar) {
            motor.set(-HOOKING_POWER);
        }
    }

    public void unhookFromBar() {
        if (onBar) {
            motor.set(HOOKING_POWER);
            onBar = false;
        }
    }

    private boolean isHooked() {
        double current = getMotorCurrent();
        if (current > MIN_RUNNING_OUTPUT_CURRENT) { // TODO do we need this check?
            if (current >= NORMAL_OUTPUT) { 
                currentAtHookedCount++;
            } else {
                currentAtHookedCount = 0;
            }
            if (currentAtHookedCount >= 4) { // 4 consecutive readings higher than peak
                return true;
            }
        }
        return false;

    }

    public boolean isHookedOnBar() {
        return onBar;
    }


    public void climb(double rotations) {
        target = rotations;
        SmartDashboard.putNumber("Climber rotation target", rotations);
        leadController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion);
    }

    public void retractToMinHeight() {
        climb(MOTOR_ROTATIONS_AT_RETRACTED);
    }

    public void climbToHeight(String barType) {
        switch(barType) {
            case "low":
                climb(LOW_BAR_TARGET_HEIGHT_CM * ROTATIONS_PER_CM);
            case "mid":
                climb(MID_BAR_TARGET_HEIGHT_CM * ROTATIONS_PER_CM);
            case "high":
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
        
    }

    @Override
    public void debugSubsystem() {

    }

}