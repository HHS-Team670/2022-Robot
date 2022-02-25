package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class Climber extends MustangSubsystemBase { // og telescoping

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;

    private double power = 0.25;

    // SmartMotion constants 
    // TODO: different for diagonal vs vertical?
    private static final double MAX_ACC = 200; // TODO test
    private static final double MIN_VEL = 0; // TODO test
    private static final double MAX_VEL = 200; // TODO test
    // public static final double MAX_EXTENDING_HEIGHT_CM;


    private static final double ALLOWED_ERR = 3;

    private static final double NORMAL_OUTPUT = 6.5; // this should be the current output when running normally

    private int SMARTMOTION_SLOT = 0;

    private SparkMaxPIDController leadController;
    private RelativeEncoder leadEncoder;

    private boolean onBar;
    private double target;

    private int currentAtHookedCount;

    public float motorRotationsAtRetracted;
    public float motorRotationsAtMaxExtension;

    private float softLimitAtRetracted;
    private float softLimitAtExtension;

    private double ROTATIONS_PER_CM; // TODO: find rotations per cm
    private double HALF_CM = ROTATIONS_PER_CM / 2;

    private SparkMAXLite motor;

    public Climber(int motorId, double p, double i, double d, double ff, float motorRotationsAtRetracted,
            float motorRotationsAtMaxExtension, double rotationsPerCM) {
        kP = p;
        kI = i;
        kD = d;
        kFF = ff;
        this.motorRotationsAtRetracted = motorRotationsAtRetracted;
        this.motorRotationsAtMaxExtension = motorRotationsAtMaxExtension;
        this.softLimitAtRetracted = this.motorRotationsAtRetracted + .5f;
        this.softLimitAtExtension = motorRotationsAtMaxExtension - 10;

        ROTATIONS_PER_CM = rotationsPerCM;
        HALF_CM = 0.5 * ROTATIONS_PER_CM;

        motor = SparkMAXFactory.buildFactorySparkMAX(motorId, Motor_Type.NEO);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, softLimitAtExtension);
        motor.setSoftLimit(SoftLimitDirection.kReverse, softLimitAtRetracted);

        leadController = motor.getPIDController();
        leadEncoder = motor.getEncoder();
        leadEncoder.setPosition(this.motorRotationsAtRetracted);
        setDefaultPID();

        onBar = false;
        target = 0.0;
        currentAtHookedCount = 0;
        SmartDashboard.putNumber("Climber power", 0.0);
        SmartDashboard.putBoolean("Climber deploy", false);
        SmartDashboard.putNumber("Climber motor rotations", 0);
        SmartDashboard.putNumber("Climber rotation target", 0);
    }

    public void setDefaultPID() {
        leadController.setP(kP);
        leadController.setI(kI);
        leadController.setD(kD);
        leadController.setFF(kFF);
        leadController.setSmartMotionMaxVelocity(MAX_VEL, this.SMARTMOTION_SLOT);
        leadController.setSmartMotionMinOutputVelocity(MIN_VEL, this.SMARTMOTION_SLOT);
        leadController.setSmartMotionMaxAccel(MAX_ACC, this.SMARTMOTION_SLOT);
        leadController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, this.SMARTMOTION_SLOT);
    }

    public void hookOnBar() {
        if (isHooked() && !onBar) {
            onBar = true;
        }

        if (!onBar) {
            setPower(-1 * power);
        }
    }

    public void unhookFromBar() {
        if (onBar) {
            setPower(power);
            onBar = false;
        }
    }

    private boolean isHooked() {
        double current = motor.getOutputCurrent();
        if (current > 0.2) {
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

    public void setPower(double newPower) {
        motor.set(newPower);
    }

    public void climb(double heightCM) {
        double rotations = heightCM * ROTATIONS_PER_CM;
        target = rotations;
        SmartDashboard.putNumber("Climber rotation target", rotations);
        leadController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion);
    }

    public HealthState checkHealth() {
        if ((motor == null || motor.getLastError() != REVLibError.kOk)) {
                return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    public boolean isAtTarget() {
        return (Math.abs(leadEncoder.getPosition() - target) < HALF_CM);
    }

    protected double getUnadjustedMotorRotations() {
        return this.leadEncoder.getPosition();
    }

    protected double getMotorCurrent() {
        return this.motor.getOutputCurrent();
    }

    public void test() {
        setPower(SmartDashboard.getNumber("Climber power", 0.0));
        SmartDashboard.putNumber("Climber motor rotations", getUnadjustedMotorRotations());
    }

    public void stop() {
        setPower(0);
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub

    }

}