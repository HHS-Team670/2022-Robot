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
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

public class ClimberSystem {

    public enum Level {
        LOW(87.3), MID(VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION), HIGH(DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION),
        INTERMEDIATE_HIGH(DIAGONAL_MOTOR_ROTATIONS_TO_PARTIAL_EXTENSION),
        INTERMEDIATE_MID(VERTICAL_MOTOR_ROTATIONS_TO_PARTIAL_EXTENSION);

        private final double rotations;

        Level(double rotations) {
            this.rotations = rotations;
        }

        /**
         * Gets the ID of the state.
         */
        public double getRotations() {
            return rotations;
        }

    }

    protected static final double ALLOWED_ERR_ROTATIONS = 0.05;

    protected static final double NORMAL_OUTPUT = 20; // this should be the current output when running normally
    protected static final int SMARTMOTION_SLOT = 0;

    protected static final double VERTICAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 24;
    protected static final double DIAGONAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 25;

    protected static final double VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    protected static final double VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 108; // from 3/1 testing
    protected static final double VERTICAL_MOTOR_ROTATIONS_TO_PARTIAL_EXTENSION = 83;

    protected static final double DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    protected static final double DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = -128;
    protected static final double DIAGONAL_MOTOR_ROTATIONS_TO_PARTIAL_EXTENSION = -60;

    // SmartMotion constants
    protected static final double MAX_ACC = 5676;
    protected static final double MAX_VEL = 8000;
    protected static final double MIN_VEL = 0;

    protected static final double VERTICAL_kFF = 0.00017618;
    protected static final double VERTICAL_kP = 0.00002;

    protected static final double DIAGONAL_kFF = 0.00017618;
    protected static final double DIAGONAL_kP = 0.00002;

    private static Climber verticalClimber, diagonalClimber;

    public ClimberSystem() {
        verticalClimber = new Climber(RobotMap.VERTICAL_CLIMBER, SMARTMOTION_SLOT, VERTICAL_kFF, VERTICAL_kP, false,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION,
                ALLOWED_ERR_ROTATIONS,
                MAX_ACC, MIN_VEL, MAX_VEL);

        diagonalClimber = new Climber(RobotMap.DIAGONAL_CLIMBER, SMARTMOTION_SLOT, DIAGONAL_kFF, DIAGONAL_kP, true,
                DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED, DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION,
                ALLOWED_ERR_ROTATIONS,
                MAX_ACC, MIN_VEL, MAX_VEL);
    }

    public Climber getVerticalClimber() {
        return verticalClimber;
    }

    public Climber getDiagonalClimber() {
        return diagonalClimber;
    }

    public class Climber extends MustangSubsystemBase {

        public static final double HOOKING_POWER = 0.3; // power used when hooking climber
        private static final int CURRENT_LIMIT = 85;

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

        private final double ALLOWED_ERROR;

        private final int MOTOR_ID;
        private final int SMARTMOTION_SLOT;

        private SparkMAXLite motor;
        private SparkMaxLimitSwitch limitSwitch;

        private boolean isReversed;
        private boolean isZeroedAtStart = false;

        /**
         * @param motorId                      The CAN id for the motor controller
         * @param ff                           The Feed Forward value
         * @param p                            The P value from PID
         * @param isReversed                   Whether the climber direction is flipped
         *                                     (negative is outwards, positive is
         *                                     inwards)
         * @param motorRotationsAtRetracted    Number of rotations when climber is all
         *                                     the way in
         * @param motorRotationsAtMaxExtension Number of rotations when climber is all
         *                                     the way out
         * @param rotationsPerCM
         * @param maxAcc                       Max acceleration the climber can take
         * @param maxVel                       Max velocity the climber can take
         * @param minVel                       Minimum velocity the climber can take
         */
        public Climber(int motorId, int smartmotionSlot, double ff, double p, boolean isReversed,
                double motorRotationsAtRetracted,
                double motorRotationsAtMaxExtension, double allowedError, double maxAcc, double minVel, double maxVel) {

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
            this.SMARTMOTION_SLOT = smartmotionSlot;
            this.isReversed = isReversed;

            motor = SparkMAXFactory.buildFactorySparkMAX(motorId, Motor_Type.NEO);
            motor.setIdleMode(IdleMode.kBrake);
            motor.enableSoftLimit(SoftLimitDirection.kForward, false);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

            motor.setSmartCurrentLimit(CURRENT_LIMIT);

            if (!isReversed) {
                limitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
                limitSwitch.enableLimitSwitch(true);
            } else {
                limitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
                limitSwitch.enableLimitSwitch(false);
            }

            leadController = motor.getPIDController();
            leadEncoder = motor.getEncoder();
            leadEncoder.setPosition(0);
            setSmartMotionConstants();

            onBar = false;
            target = 0.0;
            currentAtHookedCount = 0;
            this.ALLOWED_ERROR = allowedError;
            SmartDashboard.putNumber("Climber Target on " + MOTOR_ID + ":", target);
        }

        public void setSmartMotionConstants() {
            leadController.setFF(kFF, SMARTMOTION_SLOT);
            leadController.setP(kP, SMARTMOTION_SLOT);
            leadController.setSmartMotionMaxVelocity(MAX_VEL, SMARTMOTION_SLOT);
            leadController.setSmartMotionMinOutputVelocity(MIN_VEL, SMARTMOTION_SLOT);
            leadController.setSmartMotionMaxAccel(MAX_ACC, SMARTMOTION_SLOT);
            leadController.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR,
                    SMARTMOTION_SLOT);
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
            if (isReversed)
                power *= -1;
            motor.set(power);
        }

        private boolean isHooked() {
            double current = getMotorCurrent();
            if (current >= ClimberSystem.NORMAL_OUTPUT) {
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
            leadController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion, SMARTMOTION_SLOT);
        }

        public void retract() {
            climb(MOTOR_ROTATIONS_AT_RETRACTED);
        }

        public HealthState checkHealth() {
            if (!isZeroedAtStart) {
                return HealthState.UNKNOWN;
            }
            else{
                if ((isLimitSwitchTripped() && Math.abs(leadEncoder.getPosition()) > ALLOWED_ERROR) || (motor == null || motor.getLastError() != REVLibError.kOk)) {
                    return HealthState.RED;
                }
            }
            return HealthState.GREEN;
        }

        public boolean isAtTarget() {
            return (Math.abs(leadEncoder.getPosition() - target) < ALLOWED_ERROR);
        }

        public boolean isLimitSwitchTripped() {
            return limitSwitch.isPressed();
        }

        protected double getUnadjustedMotorRotations() {
            return this.leadEncoder.getPosition();
        }

        protected double getMotorCurrent() {
            return this.motor.getOutputCurrent();
        }

        public void stop() {
            motor.set(0);
        }

        public RelativeEncoder getLeadEncoder() {
            return leadEncoder;
        }

        @Override
        public void mustangPeriodic() {
            if (getHealth(false) == HealthState.UNKNOWN) {
                if (isLimitSwitchTripped()) {
                    leadEncoder.setPosition(0);
                    isZeroedAtStart = true;
                    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
                    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                    if (!isReversed) {
                        motor.setSoftLimit(SoftLimitDirection.kForward, (float)
                        SOFT_LIMIT_AT_EXTENSION);
                        motor.setSoftLimit(SoftLimitDirection.kReverse, (float)
                        SOFT_LIMIT_AT_RETRACTED);
                    } else {
                        motor.setSoftLimit(SoftLimitDirection.kReverse, (float)
                        SOFT_LIMIT_AT_EXTENSION);
                        motor.setSoftLimit(SoftLimitDirection.kForward, (float)
                        SOFT_LIMIT_AT_RETRACTED);
                    }
                }
            }
        }

        @Override
        public void debugSubsystem() {
            double rotations = SmartDashboard.getNumber("Climber Target on " + MOTOR_ID + ":", 0);
            SmartDashboard.putNumber("Climber Pos on " + MOTOR_ID + ":", leadEncoder.getPosition());
            climb(rotations);
        }

    }
}
