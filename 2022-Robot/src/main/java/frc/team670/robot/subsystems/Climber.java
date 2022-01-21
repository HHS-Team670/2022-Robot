package frc.team670.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;

/**
 * 
 * @author Pallavi, Eugenia, Sofia, ctychen
 */
public class Climber extends MustangSubsystemBase {

    private static final double kP = 0.00005;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kFF = 0.0120;

    // SmartMotion constants
    private static final double MAX_ACC = 200;
    private static final double MIN_VEL = 0;
    private static final double MAX_VEL = 200;

    private static final double ALLOWED_ERR = 3;

    private static final double NORMAL_OUTPUT = 6.5; // Todo: this should be the current output when running normally
    private static final double ROTATIONS_PER_CM = 50.0 / 9; // gearing is 50:1
    private static final double HALF_CM = 0.5 * ROTATIONS_PER_CM;

    private int SMARTMOTION_SLOT = 0;

    private CANPIDController controller;
    private CANEncoder encoder;
    private SparkMAXLite motor;
    
    private boolean onBar;
    private double target;

    private int currentAtHookedCount = 0;

    private static final float MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float MOTOR_ROTATIONS_AT_MAX_EXTENSION = 368;

    private static final float SOFT_LIMIT_AT_RETRACTED = MOTOR_ROTATIONS_AT_RETRACTED + .5f;
    private static final float SOFT_LIMIT_AT_EXTENSION = MOTOR_ROTATIONS_AT_MAX_EXTENSION - 10;

    public Climber() {
        motor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER_MOTOR, Motor_Type.NEO);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);
        controller = motor.getPIDController();
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, SOFT_LIMIT_AT_EXTENSION);
        motor.setSoftLimit(SoftLimitDirection.kReverse, SOFT_LIMIT_AT_RETRACTED);
        encoder = motor.getEncoder();
        encoder.setPosition(MOTOR_ROTATIONS_AT_RETRACTED);
        setDefaultPID();

        onBar = false;
        SmartDashboard.putNumber("Climber power", 0.0);
        SmartDashboard.putBoolean("Climber deploy", false);
        SmartDashboard.putNumber("Climber motor rotations", 0);
        SmartDashboard.putNumber("Climber rotation target", 0);
    }

    public void setDefaultPID() {
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setFF(kFF);
        controller.setSmartMotionMaxVelocity(MAX_VEL, this.SMARTMOTION_SLOT);
        controller.setSmartMotionMinOutputVelocity(MIN_VEL, this.SMARTMOTION_SLOT);
        controller.setSmartMotionMaxAccel(MAX_ACC, this.SMARTMOTION_SLOT);
        controller.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, this.SMARTMOTION_SLOT);
    }

    public void hookOnBar() {
        if (isHooked() && !onBar) {
            setPower(0);
            onBar = true;
        }

        if (!onBar) {
            setPower(-0.25);
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
        return this.onBar;
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void climb(double heightCM) {
        double rotations = heightCM * ROTATIONS_PER_CM;
        target = rotations;
        SmartDashboard.putNumber("Climber rotation target", rotations);
        controller.setReference(rotations, ControlType.kSmartMotion);
    }

    @Override
    public HealthState checkHealth() {
        if (isSparkMaxErrored(motor)) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    public boolean isAtTarget() {
        return Math.abs(encoder.getPosition() - target) < HALF_CM;
    }

    protected double getUnadjustedMotorRotations() {
        return this.encoder.getPosition();
    }

    protected double getMotorCurrent() {
        return this.motor.getOutputCurrent();
    }

    @Override
    public void mustangPeriodic() {
        // SmartDashboard.putNumber("Climber motor rotations", getUnadjustedMotorRotations());
        // SmartDashboard.putNumber("Climber motor current", getMotorCurrent());
    }

    public void test() {
        setPower(SmartDashboard.getNumber("Climber power", 0.0));
        SmartDashboard.putNumber("Climber motor rotations", getUnadjustedMotorRotations());
    }

}