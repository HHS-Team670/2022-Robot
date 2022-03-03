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
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class Climber extends MustangSubsystemBase { // og telescoping

    public enum Level{
        LOW, MID, HIGH
    }
    private double kFF;
    private double kP;

    private static final double HOOKING_POWER = 0.05; // TODO find this, it's the power used when hooking climber

    private final double MAX_ACC; 
    private final double MIN_VEL; 
    private final double MAX_VEL;


    private static final double ALLOWED_ERR_ROTATIONS = 3;


    private static final double MIN_RUNNING_OUTPUT_CURRENT = 0.2; // TODO find this

    // TODO find this
    private static final double NORMAL_OUTPUT = 6.5; // this should be the current output when running normally

    private static final double VERTICAL_RETRACTED_HEIGHT_CM = 93.98;

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
    //private final double HIGH_BAR_HEIGHT_CM = 192;

    private final double EXTENSION_DIST_ABOVE_BAR_CM = 15.24; // half a foot

    private final double LOW_BAR_TARGET_HEIGHT_CM = LOW_BAR_HEIGHT_CM + EXTENSION_DIST_ABOVE_BAR_CM;
    private final double MID_BAR_TARGET_HEIGHT_CM = MID_BAR_HEIGHT_CM + EXTENSION_DIST_ABOVE_BAR_CM;
    //private final double HIGH_BAR_TARGET_HEIGHT_CM = HIGH_BAR_HEIGHT_CM + EXTENSION_DIST_ABOVE_BAR_CM;

    private final double SOFT_LIMIT_AT_RETRACTED;
    private final double SOFT_LIMIT_AT_EXTENSION;

    private final double ROTATIONS_PER_CM;
    private final double ROTATIONS_HALF_CM;

    private final double MOTOR_ID;

    private SparkMAXLite motor;
    private SparkMaxLimitSwitch limitSwitch;

    public Climber(int motorId, double ff, double p, double motorRotationsAtRetracted,
            double motorRotationsAtMaxExtension, double rotationsPerCM, double maxAcc, double maxVel, double minVel) {
        kFF = ff;
        kP = p;
        this.MOTOR_ROTATIONS_AT_RETRACTED = motorRotationsAtRetracted;
        this.MOTOR_ROTATIONS_AT_MAX_EXTENSION = motorRotationsAtMaxExtension;
        this.SOFT_LIMIT_AT_RETRACTED = this.MOTOR_ROTATIONS_AT_RETRACTED + .5f;
        this.SOFT_LIMIT_AT_EXTENSION = MOTOR_ROTATIONS_AT_MAX_EXTENSION - 10;
        this.MAX_ACC = maxAcc;
        this.MAX_VEL = maxVel;
        this.MIN_VEL = minVel;
        this.MOTOR_ID = motorId;

        ROTATIONS_PER_CM = rotationsPerCM;
        ROTATIONS_HALF_CM = 0.5 * ROTATIONS_PER_CM;

        motor = SparkMAXFactory.buildFactorySparkMAX(motorId, Motor_Type.NEO);
        motor.setIdleMode(IdleMode.kBrake);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float)SOFT_LIMIT_AT_EXTENSION);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float)SOFT_LIMIT_AT_RETRACTED);

        // TODO kNormallyOpen or kNormallyClosed
        // TODO reverse switch or forward?
        limitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        limitSwitch.enableLimitSwitch(true); 

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
        leadController.setP(kP);
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
            run(-HOOKING_POWER);
        }
    }

    public void run(double power) {
        motor.set(power);
    }

    public void unhookFromBar() {
        if (onBar) {
            motor.set(HOOKING_POWER);
            onBar = false;
        }
    }

    private boolean isHooked() {
        double current = getMotorCurrent();
            if (current >= NORMAL_OUTPUT) { 
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
        if (MOTOR_ID == RobotMap.VERTICAL_CLIMBER) {
            SmartDashboard.putNumber("Vertical climber rotation target", rotations);
        } else {
            SmartDashboard.putNumber("Diagonal climber rotation target", rotations);
        }
        leadController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion);
    }



    public void retract() {
        climb(MOTOR_ROTATIONS_AT_RETRACTED);
    }

    public void climbToHeight(Level barType) {
        switch(barType) {
            case LOW:
                climb(LOW_BAR_TARGET_HEIGHT_CM * ROTATIONS_PER_CM - VERTICAL_RETRACTED_HEIGHT_CM * ROTATIONS_PER_CM);
            case MID:
                climb(MID_BAR_TARGET_HEIGHT_CM * ROTATIONS_PER_CM - VERTICAL_RETRACTED_HEIGHT_CM * ROTATIONS_PER_CM);
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

    }

}