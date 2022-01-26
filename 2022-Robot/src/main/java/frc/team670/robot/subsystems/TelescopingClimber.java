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
public class TelescopingClimber extends MustangSubsystemBase {

    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kFF = 0;

    // SmartMotion constants
    private static final double MAX_ACC = 0;
    private static final double MIN_VEL = 0;
    private static final double MAX_VEL = 0;

    private static final double ALLOWED_ERR = 0;

    private static final double NORMAL_OUTPUT = 0; // Todo: this should be the current output when running normally
    private static final double ROTATIONS_PER_CM = 0; // gearing is 50:1
    private static final double HALF_CM = 0 * ROTATIONS_PER_CM;

    private int SMARTMOTION_SLOT = 0;

    private CANPIDController[] controllers;
    private CANEncoder[] encoders;
    private SparkMAXLite[] motors;
    
    private boolean onBar;
    private double[] targets;

    private int currentAtHookedCount;

    private static float MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static float MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0;

    private static float SOFT_LIMIT_AT_RETRACTED = MOTOR_ROTATIONS_AT_RETRACTED + .5f;
    private static float SOFT_LIMIT_AT_EXTENSION = MOTOR_ROTATIONS_AT_MAX_EXTENSION - 10;

    public double MAX_EXTENDING_HEIGHT_CM; // TODO: change this later

    public TelescopingClimber(int motor1, int motor2, double[] pidConstants, float[] motorStats, double mehc) {
        motors = new SparkMAXLite[]{SparkMAXFactory.buildFactorySparkMAX(motor1, Motor_Type.NEO), SparkMAXFactory.buildFactorySparkMAX(motor2, Motor_Type.NEO)};
        for (SparkMAXLite motor : motors)
        {
            motor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.CLIMBER_MOTOR1, Motor_Type.NEO);
            motor.setIdleMode(IdleMode.kBrake);
            motor.setInverted(true); 
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
            motor.setSoftLimit(SoftLimitDirection.kForward, SOFT_LIMIT_AT_EXTENSION);
            motor.setSoftLimit(SoftLimitDirection.kReverse, SOFT_LIMIT_AT_RETRACTED);  
        }
        motors[1].follow(motors[0]);
        controllers = new CANPIDController[motors.length];
        encoders = new CANEncoder[motors.length];
        for (int i = 0; i < controllers.length; i++)
        {
            controllers[i] = motors[i].getPIDController();
        }
        for (int j = 0; j < encoders.length; j++)
        {
            encoders[j] = motors[j].getEncoder();
            encoders[j].setPosition(MOTOR_ROTATIONS_AT_RETRACTED);
        }

        kP = pidConstants[0];
        kI = pidConstants[1];
        kD = pidConstants[2];
        kFF = pidConstants[3];
        MOTOR_ROTATIONS_AT_RETRACTED = motorStats[0];
        MOTOR_ROTATIONS_AT_MAX_EXTENSION = motorStats[1];

        MAX_EXTENDING_HEIGHT_CM = mehc;

        setDefaultPID();

        onBar = false;
        targets = new double[]{0.0, 0.0};
        currentAtHookedCount = 0;
        SmartDashboard.putNumber("Climber power", 0.0);
        SmartDashboard.putBoolean("Climber deploy", false);
        SmartDashboard.putNumber("Climber motor rotations", 0);
        SmartDashboard.putNumber("Climber rotation target", 0);
    }

    public void setDefaultPID() {
        for (CANPIDController controller : controllers)
        {
            controller.setP(kP);
            controller.setI(kI);
            controller.setD(kD);
            controller.setFF(kFF);
            controller.setSmartMotionMaxVelocity(MAX_VEL, this.SMARTMOTION_SLOT);
            controller.setSmartMotionMinOutputVelocity(MIN_VEL, this.SMARTMOTION_SLOT);
            controller.setSmartMotionMaxAccel(MAX_ACC, this.SMARTMOTION_SLOT);
            controller.setSmartMotionAllowedClosedLoopError(ALLOWED_ERR, this.SMARTMOTION_SLOT);
        }
    }

    public void hookOnBar() {
        if (isHooked() && !onBar) {
            onBar = true;
        }

        if (!onBar) {
            setPower(-0.25);
        }
    }

    public void unhookFromBar() {
        if (onBar) {
            setPower(0.25);
            onBar = false;
        }
    }

    private boolean isHooked() {
        double current = motors[0].getOutputCurrent();
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

    public void setPower(double power)
    {
        motors[0].set(power);
    }

    public void climb(double heightCM) {
        double rotations = heightCM * ROTATIONS_PER_CM;
        SmartDashboard.putNumber("Climber rotation target", rotations);
        for (int i = 0; i < targets.length; i++)
        {
            targets[i] = rotations;
        }
        for (CANPIDController controller : controllers)
        {
            controller.setReference(rotations, ControlType.kSmartMotion);
        }
    }

    @Override
    public HealthState checkHealth() {
        for (SparkMAXLite motor : motors)
        {
            if (isSparkMaxErrored(motor))
            {
                return HealthState.RED;
            }
        }
        return HealthState.GREEN;
    }

    public boolean isAtTarget() {
        return (Math.abs(encoders[0].getPosition() - targets[0]) < HALF_CM) && (Math.abs(encoders[1].getPosition() - targets[1]) < HALF_CM);
    }

    protected double getUnadjustedAvgMotorRotations() {
        return (getUnadjustedMotorRotations(0) + getUnadjustedMotorRotations(1)) / 2.0;
    }

    protected double getUnadjustedMotorRotations(int mtr)
    {
        return this.encoders[mtr].getPosition();
    }

    protected double getMotorCurrent(int mtr) {
        return this.motors[mtr].getOutputCurrent();
    }

    protected double getAvgMotorCurrent2() {
        return (getMotorCurrent(0) + getMotorCurrent(1)) / 2.0;
    }


    @Override
    public void mustangPeriodic() {
        // SmartDashboard.putNumber("Climber motor rotations", getUnadjustedMotorRotations());
        // SmartDashboard.putNumber("Climber motor current", getMotorCurrent());
    }

    public void test() {
        setPower(SmartDashboard.getNumber("Climber power", 0.0));
        SmartDashboard.putNumber("Climber motor rotations", getUnadjustedAvgMotorRotations());
    }

}