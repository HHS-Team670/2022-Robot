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
 * @author Pallavi, Eugenia, Sofia, ctychen, Sanatan
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

    private CANPIDController controllerStraight;
    private CANEncoder encoderStraight;
    private SparkMAXLite motorStraight;
    private CANPIDController controllerStraight2;
    private CANEncoder encoderStraight2;
    private SparkMAXLite motorStraight2;

    private CANPIDController controllerOblique;
    private CANEncoder encoderOblique;
    private SparkMAXLite motorOblique;
    private CANPIDController controllerOblique2;
    private CANEncoder encoderOblique2;
    private SparkMAXLite motorOblique2;
    
    private boolean onBar;
    private double target;
    private boolean onBar2;
    private double target2;

    private int currentAtHookedCount = 0;
    private int currentAtHookedCount2 = 0;

    private boolean onBarOblique;
    private double targetOblique;
    private boolean onBarOblique2;
    private double targetOblique2;

    private int currentAtHookedCountOblique = 0;
    private int currentAtHookedCountOblique2 = 0;

    private static final float MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float MOTOR_ROTATIONS_AT_MAX_EXTENSION = 368;

    private static final float SOFT_LIMIT_AT_RETRACTED = MOTOR_ROTATIONS_AT_RETRACTED + .5f;
    private static final float SOFT_LIMIT_AT_EXTENSION = MOTOR_ROTATIONS_AT_MAX_EXTENSION - 10;

    public Climber() {
        motorStraight = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER_MOTOR1, Motor_Type.NEO);
        motorStraight.setIdleMode(IdleMode.kBrake);
        motorStraight.setInverted(true);
        controllerStraight = motorStraight.getPIDController();
        motorStraight.enableSoftLimit(SoftLimitDirection.kForward, true);
        motorStraight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motorStraight.setSoftLimit(SoftLimitDirection.kForward, SOFT_LIMIT_AT_EXTENSION);
        motorStraight.setSoftLimit(SoftLimitDirection.kReverse, SOFT_LIMIT_AT_RETRACTED);
        encoderStraight = motorStraight.getEncoder();
        encoderStraight.setPosition(MOTOR_ROTATIONS_AT_RETRACTED);
        setDefaultPID();
        motorStraight2 = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER_MOTOR2, Motor_Type.NEO);
        motorStraight2.setIdleMode(IdleMode.kBrake);
        motorStraight2.setInverted(true);
        controllerStraight2 = motorStraight2.getPIDController();
        motorStraight2.enableSoftLimit(SoftLimitDirection.kForward, true);
        motorStraight2.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motorStraight2.setSoftLimit(SoftLimitDirection.kForward, SOFT_LIMIT_AT_EXTENSION);
        motorStraight2.setSoftLimit(SoftLimitDirection.kReverse, SOFT_LIMIT_AT_RETRACTED);
        encoderStraight2 = motorStraight2.getEncoder();
        encoderStraight2.setPosition(MOTOR_ROTATIONS_AT_RETRACTED);
        setDefaultPID();


        motorOblique = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER_MOTOR3, Motor_Type.NEO);
        motorStraight.setIdleMode(IdleMode.kBrake);
        motorOblique.setInverted(true);
        controllerOblique = motorOblique.getPIDController();
        motorOblique.enableSoftLimit(SoftLimitDirection.kForward, true);
        motorOblique.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motorOblique.setSoftLimit(SoftLimitDirection.kForward, SOFT_LIMIT_AT_EXTENSION);
        motorOblique.setSoftLimit(SoftLimitDirection.kReverse, SOFT_LIMIT_AT_RETRACTED);
        encoderOblique = motorOblique.getEncoder();
        encoderOblique.setPosition(MOTOR_ROTATIONS_AT_RETRACTED);
        setDefaultPID();
        motorOblique2 = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER_MOTOR4, Motor_Type.NEO);
        motorStraight2.setIdleMode(IdleMode.kBrake);
        motorOblique2.setInverted(true);
        controllerOblique2 = motorOblique2.getPIDController();
        motorOblique2.enableSoftLimit(SoftLimitDirection.kForward, true);
        motorOblique2.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motorOblique2.setSoftLimit(SoftLimitDirection.kForward, SOFT_LIMIT_AT_EXTENSION);
        motorOblique2.setSoftLimit(SoftLimitDirection.kReverse, SOFT_LIMIT_AT_RETRACTED);
        encoderOblique2 = motorOblique2.getEncoder();
        encoderOblique2.setPosition(MOTOR_ROTATIONS_AT_RETRACTED);
        setDefaultPID();

        onBar = false;
        onBar2 = false;
        onBarOblique = false;
        onBarOblique2 = false;
        SmartDashboard.putNumber("Climber power", 0.0);
        SmartDashboard.putBoolean("Climber deploy", false);
        SmartDashboard.putNumber("Climber motor rotations", 0);
        SmartDashboard.putNumber("Climber rotation target", 0);
    }

    public void setDefaultPID() {
        controllerOblique.setP(kP);
        controllerOblique.setI(kI);
        controllerOblique.setD(kD);
        controllerOblique.setFF(kFF);
        controllerOblique.setSmartMotionMaxVelocity(MAX_VEL, this.SMARTMOTION_SLOT);
        controllerOblique.setSmartMotionMinOutputVelocity(MIN_VEL, this.SMARTMOTION_SLOT);
        controllerOblique.setSmartMotionMaxAccel(MAX_ACC, this.SMARTMOTION_SLOT);
        controllerOblique.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, this.SMARTMOTION_SLOT);

        controllerStraight.setP(kP);
        controllerStraight.setI(kI);
        controllerStraight.setD(kD);
        controllerStraight.setFF(kFF);
        controllerStraight.setSmartMotionMaxVelocity(MAX_VEL, this.SMARTMOTION_SLOT);
        controllerStraight.setSmartMotionMinOutputVelocity(MIN_VEL, this.SMARTMOTION_SLOT);
        controllerStraight.setSmartMotionMaxAccel(MAX_ACC, this.SMARTMOTION_SLOT);
        controllerStraight.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, this.SMARTMOTION_SLOT);
    }

    public void hookOnBar() {
        if (isHooked() && !onBar) {
            setPower1(0);
            onBar = true;
        }

        if (isHooked2() && !onBar2) {
            setPower2(0);
            onBar2 = true;
        }

        if (!onBar) {
            setPower1(-0.25);
        }
        if (!onBar2) {
            setPower2(-0.25);
        }
    }

    public void unhookFromBar() {
        if (onBar) {
            setPower1(0.25);
            onBar = false;
        }

        if (onBar2) {
            setPower2(0.25);
            onBar2 = false;
        }
    }

    private boolean isHooked() {
        double current = motorStraight.getOutputCurrent();
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

    private boolean isHooked2() {
        double current = motorStraight2.getOutputCurrent();
        if (current > 0.2) {
            if (current >= NORMAL_OUTPUT) {
                currentAtHookedCount2++;
            } else {
                currentAtHookedCount2 = 0;
            }
            if (currentAtHookedCount2 >= 4) { // 4 consecutive readings higher than peak
                return true;
            }
        }
        return false;

    }

    public boolean isHookedOnBar() {
        return this.onBar;
    }
    public boolean isHookedOnBar2() {
        return this.onBar2;
    }

    public void setPower1(double power) {
        motorStraight.set(power);
    }
    public void setPower2(double power) {
        motorStraight2.set(power);
    }

    public void climb(double heightCM) {
        double rotations = heightCM * ROTATIONS_PER_CM;
        target = rotations;
        target2 = rotations;
        SmartDashboard.putNumber("Climber rotation target", rotations);
        controllerStraight.setReference(rotations, ControlType.kSmartMotion);
        controllerStraight2.setReference(rotations, ControlType.kSmartMotion);
    }

    @Override
    public HealthState checkHealth() {
        if (isSparkMaxErrored(motorStraight) || isSparkMaxErrored(motorOblique) || isSparkMaxErrored(motorStraight2) || isSparkMaxErrored(motorOblique2)) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    public boolean isAtTarget() {
        return Math.abs(encoderStraight.getPosition() - target) < HALF_CM;
    }

    public boolean isAtTarget2() {
        return Math.abs(encoderStraight2.getPosition() - target2) < HALF_CM;
    }

    protected double getUnadjustedMotorRotations() {
        return this.encoderStraight.getPosition();
    }

    protected double getMotorCurrent() {
        return this.motorStraight.getOutputCurrent();
    }

    protected double getUnadjustedMotorRotations2() {
        return this.encoderStraight2.getPosition();
    }

    protected double getMotorCurrent2() {
        return this.motorStraight2.getOutputCurrent();
    }

    public void hookOnBarOblique() {
        if (isHookedOblique() && !onBarOblique) {
            setPowerOblique1(0);
            onBarOblique = true;
        }

        if (isHookedOblique2() && !onBarOblique2) {
            setPowerOblique2(0);
            onBarOblique2 = true;
        }

        if (!onBarOblique) {
            setPowerOblique1(-0.25);
        }
        if (!onBarOblique2) {
            setPowerOblique2(-0.25);
        }
    }

    private boolean isHookedOblique() {
        double current = motorOblique.getOutputCurrent();
        if (current > 0.2) {
            if (current >= NORMAL_OUTPUT) {
                currentAtHookedCountOblique++;
            } else {
                currentAtHookedCountOblique = 0;
            }
            if (currentAtHookedCountOblique >= 4) { // 4 consecutive readings higher than peak
                return true;
            }
        }
        return false;

    }

    private boolean isHookedOblique2() {
        double current = motorOblique2.getOutputCurrent();
        if (current > 0.2) {
            if (current >= NORMAL_OUTPUT) {
                currentAtHookedCountOblique2++;
            } else {
                currentAtHookedCountOblique2 = 0;
            }
            if (currentAtHookedCountOblique2 >= 4) { // 4 consecutive readings higher than peak
                return true;
            }
        }
        return false;

    }

    public boolean isHookedOnBarOblique() {
        return this.onBarOblique;
    }

    public boolean isHookedOnBarOblique2() {
        return this.onBarOblique2;
    }

    public void setPowerOblique1(double power) {
        motorOblique.set(power);
    }

    public void setPowerOblique2(double power) {
        motorOblique2.set(power);
    }

    public void climbOblique(double heightCM) {
        double rotations = heightCM * ROTATIONS_PER_CM;
        targetOblique = rotations;
        targetOblique2 = rotations;
        SmartDashboard.putNumber("Climber rotation target", rotations);
        controllerOblique.setReference(rotations, ControlType.kSmartMotion);
        controllerOblique2.setReference(rotations, ControlType.kSmartMotion);
    }

    public boolean isAtTargetOblique() {
        return Math.abs(encoderOblique.getPosition() - targetOblique) < HALF_CM;
    }

    protected double getUnadjustedMotorRotationsOblique() {
        return this.encoderOblique.getPosition();
    }

    protected double getMotorCurrentOblique() {
        return this.motorOblique.getOutputCurrent();
    }

    public boolean isAtTargetOblique2() {
        return Math.abs(encoderOblique2.getPosition() - targetOblique2) < HALF_CM;
    }

    protected double getUnadjustedMotorRotationsOblique2() {
        return this.encoderOblique2.getPosition();
    }

    protected double getMotorCurrentOblique2() {
        return this.motorOblique2.getOutputCurrent();
    }


    @Override
    public void mustangPeriodic() {
        // SmartDashboard.putNumber("Climber motor rotations", getUnadjustedMotorRotations());
        // SmartDashboard.putNumber("Climber motor current", getMotorCurrent());
    }

    public void test() {
        setPower1(SmartDashboard.getNumber("Climber power", 0.0));
        SmartDashboard.putNumber("Climber motor rotations", getUnadjustedMotorRotations());
    }

}