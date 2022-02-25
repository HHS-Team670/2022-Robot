package frc.team670.robot.subsystems;

import java.util.ArrayList;

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
import frc.team670.robot.constants.RobotMap;

/**
 * 
 * @author Pallavi, ctychen, Sanatan
 */
public class ClimberSystem extends MustangSubsystemBase {

    private static final double CLIMBER1_kP = 0;
    private static final double CLIMBER1_kI = 0;
    private static final double CLIMBER1_kD = 0;
    private static final double CLIMBER1_kFF = 0;

    private static final float CLIMBER1_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float CLIMBER1_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0;

    private static final double CLIMBER2_kP = 0;
    private static final double CLIMBER2_kI = 0;
    private static final double CLIMBER2_kD = 0;
    private static final double CLIMBER2_kFF = 0;

    private static final float CLIMBER2_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float CLIMBER2_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0;

    private Climber climber1; // VERTICAL
    private Climber climber2; // DIAGONAL

    public ClimberSystem() {
        climber1 = new Climber(RobotMap.CLIMBER_ONE, CLIMBER1_kP, CLIMBER1_kI, CLIMBER1_kD, CLIMBER1_kFF,
                CLIMBER1_MOTOR_ROTATIONS_AT_RETRACTED, CLIMBER1_MOTOR_ROTATIONS_AT_MAX_EXTENSION, 66.24);
        climber2 = new Climber(RobotMap.CLIMBER_TWO, CLIMBER2_kP, CLIMBER2_kI, CLIMBER2_kD, CLIMBER2_kFF,
                CLIMBER2_MOTOR_ROTATIONS_AT_RETRACTED, CLIMBER2_MOTOR_ROTATIONS_AT_MAX_EXTENSION, 66.24);
    }

    public void stop() {
        climber1.setPower(0.0);
        climber2.setPower(0.0);
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        if (climber1.checkHealth() == HealthState.GREEN && climber2.checkHealth() == HealthState.GREEN) {
            return HealthState.GREEN;
        }
        return HealthState.RED;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub
    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub

    }

    public MustangSubsystemBase getClimber1() {
        return climber1;
    }

    public MustangSubsystemBase getClimber2() {
        return climber2;
    }

    public boolean isAtTarget(boolean vertical) {
        if (vertical) {
            return climber1.isAtTarget();
        } else {
            return climber2.isAtTarget();
        } 
    }

    public void climb(boolean vertical, double heightCM) {
        if (vertical) {
            climber1.climb(heightCM);
        } else {
            climber2.climb(heightCM);
        }
    }
}

class Climber extends MustangSubsystemBase { // og telescoping

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;

    private double power = 0.25;

    // SmartMotion constants
    private static final double MAX_ACC = 0;
    private static final double MIN_VEL = 0;
    private static final double MAX_VEL = 0;

    private static final double ALLOWED_ERR = 0;

    private static final double NORMAL_OUTPUT = 0; // Todo: this should be the current output when running normally
    private static final double ROTATIONS_PER_CM = 0; // gearing is 50:1
    private static final double HALF_CM = 0.5 * ROTATIONS_PER_CM;

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

    public double MAX_EXTENDING_HEIGHT_CM; // TODO: change this later
    private SparkMAXLite motor;

    public Climber(int motorId, double p, double i, double d, double ff, float motorRotationsAtRetracted,
            float motorRotationsAtMaxExtension, double maxExtendingHeightCm) {
        kP = p;
        kI = i;
        kD = d;
        kFF = ff;
        this.motorRotationsAtRetracted = motorRotationsAtRetracted;
        this.motorRotationsAtMaxExtension = motorRotationsAtMaxExtension;
        this.softLimitAtRetracted = this.motorRotationsAtRetracted + .5f;
        this.softLimitAtExtension = motorRotationsAtMaxExtension - 10;

        MAX_EXTENDING_HEIGHT_CM = maxExtendingHeightCm;

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

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub

    }

}