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

    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kFF = 0;

    // SmartMotion constants
    private static final double MAX_ACC = 0;
    private static final double MIN_VEL = 0;
    private static final double MAX_VEL = 0;

    private static final double ALLOWED_ERR = 0;

    private static final double NORMAL_OUTPUT = 0; // Todo: this should be the current output when running normally
    private static final double ROTATIONS_PER_CM = 0; // gearing is 50:1
    private static final double HALF_CM = 0 * ROTATIONS_PER_CM;

    //oblique:

    private static final double kPO = 0;
    private static final double kIO = 0;
    private static final double kDO = 0;
    private static final double kFFO = 0;

    // SmartMotion constants
    private static final double MAX_ACCO = 0;
    private static final double MIN_VELO = 0;
    private static final double MAX_VELO = 0;

    private static final double ALLOWED_ERRO = 0;

    private static final double NORMAL_OUTPUTO = 0; // Todo: this should be the current output when running normally
    private static final double ROTATIONS_PER_CMO = 0 / 9; // gearing is 50:1
    private static final double HALF_CMO = 0 * ROTATIONS_PER_CM;

    private int SMARTMOTION_SLOT = 0;
    private int SMARTMOTION_SLOTO = 0;

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
    private double pow1, pow2;

    private boolean onBarOblique;
    private double targetOblique;
    private boolean onBarOblique2;
    private double targetOblique2;

    private int currentAtHookedCountOblique = 0;
    private int currentAtHookedCountOblique2 = 0;
    private double powOblique1, powOblique2;

    private static final float MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0;

    private static final float SOFT_LIMIT_AT_RETRACTED = MOTOR_ROTATIONS_AT_RETRACTED + .5f;
    private static final float SOFT_LIMIT_AT_EXTENSION = MOTOR_ROTATIONS_AT_MAX_EXTENSION - 10;

    private static final float MOTOR_ROTATIONS_AT_RETRACTEDO = 0;
    private static final float MOTOR_ROTATIONS_AT_MAX_EXTENSIONO = 0;

    private static final float SOFT_LIMIT_AT_RETRACTEDO = MOTOR_ROTATIONS_AT_RETRACTEDO + .5f;
    private static final float SOFT_LIMIT_AT_EXTENSIONO = MOTOR_ROTATIONS_AT_MAX_EXTENSIONO - 10;

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
        controllerOblique.setP(kPO);
        controllerOblique.setI(kIO);
        controllerOblique.setD(kDO);
        controllerOblique.setFF(kFFO);
        controllerOblique.setSmartMotionMaxVelocity(MAX_VELO, this.SMARTMOTION_SLOTO);
        controllerOblique.setSmartMotionMinOutputVelocity(MIN_VELO, this.SMARTMOTION_SLOTO);
        controllerOblique.setSmartMotionMaxAccel(MAX_ACCO, this.SMARTMOTION_SLOTO);
        controllerOblique.setSmartMotionAllowedClosedLoopError(ALLOWED_ERRO, this.SMARTMOTION_SLOTO);

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
        pow1 = power;
    }
    public void setPower2(double power) {
        motorStraight2.set(power);
        pow2 = power;
    }

    public void setPower(double power)
    {
        setPower1(power);
        setPower2(power);
    }

    public void climb(double heightCM) {
        if (heightCM < 1)
        {
            double rotations = heightCM * ROTATIONS_PER_CM;
            target = rotations;
            target2 = rotations;
            SmartDashboard.putNumber("Climber rotation target", rotations);
            controllerStraight.setReference(rotations, ControlType.kSmartMotion);
            controllerStraight2.setReference(rotations, ControlType.kSmartMotion);
            if (encoderStraight.getPosition() - encoderStraight2.getPosition() > 0)
            {
               setPower2(pow2 + (encoderStraight.getPosition() - encoderStraight2.getPosition()) / 4);
            }  
            if (encoderStraight.getPosition() - encoderStraight2.getPosition() < 0)
            {
                setPower2(pow1 + (encoderStraight2.getPosition() - encoderStraight.getPosition()) / 4);
            }
        }
        else
        {
            for (int i = 0; i < 50; i++)
            {
                climb((heightCM - 5) / 50 + 5);
            }
        }
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
            if (current >= NORMAL_OUTPUTO) {
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
            if (current >= NORMAL_OUTPUTO) {
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
        powOblique1 = power;
    }

    public void setPowerOblique2(double power) {
        motorOblique2.set(power);
        powOblique2 = power;
    }

    public void setPowerOblique(double power) {
        setPowerOblique1(power);
        setPowerOblique2(power);
    }

    public void climbOblique(double heightCM) {
        if (heightCM < 1)
        {
            double rotations = heightCM * ROTATIONS_PER_CMO;
            targetOblique = rotations;
            targetOblique2 = rotations;
            SmartDashboard.putNumber("Climber rotation target", rotations);
            controllerOblique.setReference(rotations, ControlType.kSmartMotion);
            controllerOblique2.setReference(rotations, ControlType.kSmartMotion);
            if (encoderOblique.getPosition() - encoderOblique2.getPosition() > 0)
            {
               setPower2(powOblique2 + (encoderOblique.getPosition() - encoderOblique2.getPosition()) / 4);
            }  
            if (encoderOblique.getPosition() - encoderOblique2.getPosition() < 0)
            {
                setPower2(powOblique1 + (encoderOblique2.getPosition() - encoderOblique.getPosition()) / 4);
            }
        }
        else
        {
            for (int i = 0; i < 50; i++)
            {
                climb((heightCM - 5) / 50 + 5);
            }
        }
    }

    public boolean isAtTargetOblique() {
        return Math.abs(encoderOblique.getPosition() - targetOblique) < HALF_CMO;
    }

    protected double getUnadjustedMotorRotationsOblique() {
        return this.encoderOblique.getPosition();
    }

    protected double getMotorCurrentOblique() {
        return this.motorOblique.getOutputCurrent();
    }

    public boolean isAtTargetOblique2() {
        return Math.abs(encoderOblique2.getPosition() - targetOblique2) < HALF_CMO;
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