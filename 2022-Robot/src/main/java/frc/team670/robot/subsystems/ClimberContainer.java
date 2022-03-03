package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;

public class ClimberContainer {

    public static final double HOOKING_POWER = 0.3; // power used when hooking climber

    public static final double ALLOWED_ERR_ROTATIONS = 3;

    // TODO find this, hopefully same for both c1 and c2?
    public static final double NORMAL_OUTPUT = 6.5; // this should be the current output when running normally

    public static final double VERTICAL_STARTING_HEIGHT_CM = 93.98;

    public static final int SMARTMOTION_SLOT = 0;
    public static final double LOW_BAR_HEIGHT_CM = 124;
    public static final double MID_BAR_HEIGHT_CM = 153;
    // private final double HIGH_BAR_HEIGHT_CM = 192;

    // TODO find this, it's the offset above bar
    public static final double EXTENSION_DIST_ABOVE_BAR_CM = 15.24; // half a foot

    public static final double LOW_BAR_TARGET_HEIGHT_CM = LOW_BAR_HEIGHT_CM + EXTENSION_DIST_ABOVE_BAR_CM;
    public static final double MID_BAR_TARGET_HEIGHT_CM = MID_BAR_HEIGHT_CM + EXTENSION_DIST_ABOVE_BAR_CM;
    // private final double HIGH_BAR_TARGET_HEIGHT_CM = HIGH_BAR_HEIGHT_CM +
    // EXTENSION_DIST_ABOVE_BAR_CM;

    public static final double SPOOL_DIAMETER_CM = 3.81;
    public static final double SPOOL_CIRCUMFERENCE_CM = SPOOL_DIAMETER_CM * Math.PI;

    public static final double VERTICAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 24;
    public static final double DIAGONAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 25; 

    public static final double VERTICAL_ROTATIONS_PER_CM = VERTICAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION
            / SPOOL_CIRCUMFERENCE_CM;
    public static final double VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    public static final double VERTICAL_CLIMBER_CM_TO_FULL_EXTENSION = 58.42;

    public static final double VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 108; // actually 109, from 3/1 testing

    // SmartMotion constants
    public static final double MAX_ACC = 5676;
    public static final double MAX_VEL = 5676;
    public static final double MIN_VEL = 0;

    public static final double VERTICAL_kFF = 0.000176;
    public static final double VERTICAL_kP = 0.00001;

    public static final double DIAGONAL_kFF = 0.000176;
    public static final double DIAGONAL_kP = 0.00001;

    public static final double DIAGONAL_ROTATIONS_PER_CM = DIAGONAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION
            / SPOOL_CIRCUMFERENCE_CM;
    public static final double DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    public static final double DIAGONAL_CLIMBER_CM_TO_FULL_EXTENSION = 79.2325;
    public static final double DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = -128;

    // TODO find this, it's basically how much gap you want to leave when partially
    // deploying the diagonal climber at the same time as vertical to save time
    public static final double REMAINING_DIST_TO_HIGH_CM = 15.24; // half a foot
    public static final double MOTOR_ROTATIONS_TO_PARTIAL_EXTENSION = DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION
            - DIAGONAL_ROTATIONS_PER_CM * REMAINING_DIST_TO_HIGH_CM;

    public static Climber getVerticalClimber() {
        Climber verticalClimber = new Climber(RobotMap.VERTICAL_CLIMBER, VERTICAL_kFF, VERTICAL_kP, false,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION,
                VERTICAL_ROTATIONS_PER_CM,
                MAX_ACC, MIN_VEL, MAX_VEL);
        return verticalClimber;
    }

    public static Climber getDiagonalClimber() {
        Climber diagonalClimber = new Climber(RobotMap.DIAGONAL_CLIMBER, DIAGONAL_kFF, DIAGONAL_kP, true,
                DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED, DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION,
                DIAGONAL_ROTATIONS_PER_CM,
                MAX_ACC, MIN_VEL, MAX_VEL);
        return diagonalClimber;
    }
}
