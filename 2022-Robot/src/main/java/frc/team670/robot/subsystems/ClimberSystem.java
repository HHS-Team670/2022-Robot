package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.commands.routines.climb.FullClimb;
import frc.team670.robot.constants.RobotMap;

public class ClimberSystem extends MustangSubsystemBase {

    public enum Level {
        RETRACTED(0.1), LOW(87.3), MID(VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION),
        HIGH(DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION),
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

    protected static final double ALLOWED_ERR_POSITION = 1.5; // Position ranges from 0 (minimum extension) to about 108
                                                              // for vertical climber, and more for diagonal.

    protected static final double NORMAL_OUTPUT = 20; // this should be the current output when running normally
    protected static final int SMARTMOTION_SLOT = 0;

    protected static final double VERTICAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 24;
    protected static final double DIAGONAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 25;

    protected static final double VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED = 1;
    protected static final double VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 109; // from 3/6 testing
    protected static final double VERTICAL_MOTOR_ROTATIONS_TO_PARTIAL_EXTENSION = 83;

    protected static final double DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED = 1;
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

    private MustangController mController;

    private double defaultInitedCounter = 0;

    private Deployer deployer;

    public ClimberSystem(MustangController mController, Deployer deployer) {
        verticalClimber = new Climber(RobotMap.VERTICAL_CLIMBER, SMARTMOTION_SLOT, VERTICAL_kFF, VERTICAL_kP, false,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION,
                ALLOWED_ERR_POSITION,
                MAX_ACC, MIN_VEL, MAX_VEL);

        diagonalClimber = new Climber(RobotMap.DIAGONAL_CLIMBER, SMARTMOTION_SLOT, DIAGONAL_kFF, DIAGONAL_kP, true,
                DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED, DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION,
                ALLOWED_ERR_POSITION,
                MAX_ACC, MIN_VEL, MAX_VEL);

        verticalClimber.setName("Climber 1");
        diagonalClimber.setName("Climber 2");

        this.mController = mController;
        this.deployer = deployer;
    }

    public Climber getVerticalClimber() {
        return verticalClimber;
    }

    public Climber getDiagonalClimber() {
        return diagonalClimber;
    }

    public boolean isRobotClimbing() {
        return (verticalClimber.getCurrentLevel().rotations > Level.RETRACTED.rotations
                || Math.abs(diagonalClimber.getCurrentLevel().rotations) > Level.RETRACTED.rotations);
    }

    @Override
    public HealthState checkHealth() {
        HealthState verticalClimberHealth = verticalClimber.checkHealth();
        if (verticalClimberHealth == HealthState.GREEN
                && diagonalClimber.checkHealth().getId() >= HealthState.YELLOW.getId()) {
            return HealthState.YELLOW;
        } else {
            return verticalClimberHealth;
        }
    }

    public boolean isRunning() {
        return (verticalClimber.isRunning() || diagonalClimber.isRunning());
    }

    public void climbProcedure(int step) {
        switch (step) {
            case 0:
                verticalClimber.retract();
                diagonalClimber.retract();
                break;
            case 1:
                deployer.deploy(false);
                verticalClimber.climb(Level.MID);
                diagonalClimber.retract();
                break;
            case 2:
                verticalClimber.retract();
                diagonalClimber.retract();
                break;
            case 3:
                diagonalClimber.climb(Level.HIGH);
                verticalClimber.retract();
                break;
            case 4:
                verticalClimber.climb(Level.INTERMEDIATE_MID);
                diagonalClimber.climb(Level.HIGH);
                break;
            case 5:
                verticalClimber.retract();
                diagonalClimber.climb(Level.HIGH);
                break;
            default:
                break;
        }
    }

    @Override
    public void mustangPeriodic() {
        SmartDashboard.putNumber("ClimberSystem periodic is called", System.currentTimeMillis());
        if (verticalClimber.checkHealth() == HealthState.GREEN && diagonalClimber.checkHealth() == HealthState.GREEN
                && defaultInitedCounter % 100 == 0) {
            initDefaultCommand();
        }
        defaultInitedCounter++;
    }

    @Override
    public void debugSubsystem() {

    }

    public void initDefaultCommand() {
        MustangScheduler.getInstance().setDefaultCommand(this, new FullClimb(this, deployer, mController));
    }

    public class Climber extends MustangSubsystemBase {

        public static final double HOOKING_POWER = 0.2; // power used when hooking climber
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

        private double forwardLimit;
        private double reverseLimit;

        private SparkMAXLite motor;
        private SparkMaxLimitSwitch limitSwitch;

        private boolean isReversed;
        private boolean isZeroed = false;

        private Level currentLevel = Level.RETRACTED;

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
            this.SOFT_LIMIT_AT_RETRACTED = this.MOTOR_ROTATIONS_AT_RETRACTED + .1;
            this.SOFT_LIMIT_AT_EXTENSION = MOTOR_ROTATIONS_AT_MAX_EXTENSION - 1;
            this.MAX_ACC = maxAcc;
            this.MAX_VEL = maxVel;
            this.MIN_VEL = minVel;
            this.MOTOR_ID = motorId;
            this.SMARTMOTION_SLOT = smartmotionSlot;
            this.isReversed = isReversed;

            motor = SparkMAXFactory.buildFactorySparkMAX(motorId, Motor_Type.NEO);
            motor.setIdleMode(IdleMode.kBrake);

            // soft limits are intially disabled
            motor.enableSoftLimit(SoftLimitDirection.kForward, false);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

            motor.setSmartCurrentLimit(CURRENT_LIMIT);

            if (!isReversed) {
                limitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
                limitSwitch.enableLimitSwitch(true);
            } else {
                limitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
                limitSwitch.enableLimitSwitch(true);
            }

            leadController = motor.getPIDController();
            leadEncoder = motor.getEncoder();
            leadEncoder.setPosition(0);
            setSmartMotionConstants();

            onBar = false;
            target = 0;
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

        private boolean isRunning() {
            return leadEncoder.getVelocity() > 0;
        }

        public Level getCurrentLevel() {
            return currentLevel;
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

        public boolean isZeroed() {
            return isZeroed;
        }

        private void climb(double rotations) {
            target = rotations;
            leadController.setReference(rotations, CANSparkMax.ControlType.kSmartMotion, SMARTMOTION_SLOT);
        }

        public void climb(Level level) {
            climb(level.getRotations());
            currentLevel = level;
        }

        public void retract() {
            climb(MOTOR_ROTATIONS_AT_RETRACTED);
            currentLevel = Level.RETRACTED;
        }

        public HealthState checkHealth() {
            if (!isZeroed) {
                return HealthState.UNKNOWN;
            } else {
                if ((isLimitSwitchTripped() && currentLevel != Level.RETRACTED
                        && Math.abs(leadEncoder.getPosition()) > ALLOWED_ERROR)
                        || (motor == null || motor.getLastError() != REVLibError.kOk)) {
                    return HealthState.RED;
                } 
                
                // else if (currentLevel == Level.RETRACTED && !isLimitSwitchTripped()) { // TODO: test this on robot
                //     return HealthState.UNKNOWN;
                // }
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

        public double getMinMotorPosition() {
            return MOTOR_ROTATIONS_AT_RETRACTED;
        }

        public double getMaxMotorPosition() {
            return MOTOR_ROTATIONS_AT_MAX_EXTENSION;
        }

        @Override
        public void mustangPeriodic() {
            debugSubsystem();

            // if healthstate is unknown, zeroes
            if (getHealth(false) == HealthState.UNKNOWN) {
                if (isLimitSwitchTripped()) {
                    stop();
                    leadEncoder.setPosition(0);

                    // after zeroing, if the position is less than 0.5, enables soft limits
                    if (Math.abs(leadEncoder.getPosition()) < 0.5) {
                        // motor.enableSoftLimit(SoftLimitDirection.kForward, true);
                        // motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                        // if (!isReversed) {
                        // forwardLimit = SOFT_LIMIT_AT_EXTENSION;
                        // reverseLimit = SOFT_LIMIT_AT_RETRACTED;

                        // } else {
                        // forwardLimit = SOFT_LIMIT_AT_RETRACTED;
                        // reverseLimit = SOFT_LIMIT_AT_EXTENSION;
                        // }
                        // motor.setSoftLimit(SoftLimitDirection.kForward, (float) forwardLimit);
                        // motor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverseLimit);
                        // if (Math.abs(motor.getSoftLimit(SoftLimitDirection.kForward) - forwardLimit)
                        // < 0.5
                        // && (Math.abs(motor.getSoftLimit(SoftLimitDirection.kReverse) - reverseLimit)
                        // < 0.5)) {
                        // isZeroed = true;
                        // currentLevel = Level.RETRACTED;
                        // }

                        // isZeroed = true;
                        // currentLevel = Level.RETRACTED;
                    }
                    isZeroed = true;
                    currentLevel = Level.RETRACTED;

                } else {
                    run(-ClimberSystem.Climber.HOOKING_POWER);
                }
            }
        }

        @Override
        public void debugSubsystem() {
            SmartDashboard.putNumber("Climber Pos on " + MOTOR_ID + ":", leadEncoder.getPosition());
            double rotations = SmartDashboard.getNumber("Climber Target on " + MOTOR_ID + ":", -1); // for this to work,
                                                                                                    // uncomment
                                                                                                    // "SmartDashboard.putNumber("Climber
                                                                                                    // Target on " +
                                                                                                    // MOTOR_ID + ":",
                                                                                                    // target);"
                                                                                                    // at the end of the
                                                                                                    // Climber
                                                                                                    // constructor
            SmartDashboard.putBoolean("Climber LimitSwitch for motor id " + MOTOR_ID, isLimitSwitchTripped());
            SmartDashboard.putNumber("Climber Pos on " + MOTOR_ID + ":", leadEncoder.getPosition());
            if (MOTOR_ID == 2)
                SmartDashboard.putNumber("C: isDebugSubsystem being called?", System.currentTimeMillis());
        }

    }
}
