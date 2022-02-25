package frc.team670.robot.subsystems;

import java.util.ArrayList;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.constants.RobotMap;

/**
 * 
 * @author Pallavi, ctychen, Sanatan, Aaditya, Ethan
 */
public class ClimberSystem extends MustangSubsystemBase {

    //TODO: find PIDF values for vertical climbers
    private static final double VERTICAL_kP = 0;
    private static final double VERTICAL_kI = 0;
    private static final double VERTICAL_kD = 0;
    private static final double VERTICAL_kFF = 0;
    private static final double VERTICAL_ROTATIONS_PER_CM = 2.0051;

    // circum of spool is 11.9694680102  in cm 

        // for every 24 motor rotations, the vertical spool rotates once

        // rotations per cm of vertical: 2.00510164525

        // for every 25 motor rotations, diagonal climber spool in back rotates once

        // rotations per cm of diagonal: 2.08864754713

    private static final float VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0; //TODO: talk to mech to find max motor rotations for full extension


    //TODO: find PIDF values for diagonal climbers
    private static final double DIAGONAL_kP = 0;
    private static final double DIAGONAL_kI = 0;
    private static final double DIAGONAL_kD = 0;
    private static final double DIAGONAL_kFF = 0;

    private static final float DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0; //TODO: talk to mech to find max motor rotations for full extension
    private static final double DIAGONAL_ROTATIONS_PER_CM = 2.0886;

    private Climber verticalClimber;
    private Climber diagonalClimber;

    public ClimberSystem() {

            verticalClimber = new Climber(RobotMap.VERTICAL_CLIMBER, VERTICAL_kP, VERTICAL_kI, VERTICAL_kD, VERTICAL_kFF,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, VERTICAL_ROTATIONS_PER_CM);   
        
            diagonalClimber = new Climber(RobotMap.DIAGONAL_CLIMBER, DIAGONAL_kP, DIAGONAL_kI, DIAGONAL_kD, DIAGONAL_kFF,
                DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED, DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, DIAGONAL_ROTATIONS_PER_CM);
    }

    public void stop() {
        verticalClimber.stop();
        diagonalClimber.stop();
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        if (verticalClimber.checkHealth() == HealthState.GREEN && diagonalClimber.checkHealth() == HealthState.GREEN){
            return HealthState.GREEN;
        }
        return HealthState.RED;
    }

    @Override
    public void mustangPeriodic() {
    }

    @Override
    public void debugSubsystem() {
    }

    public Climber getVerticalClimbers() {
        return verticalClimber;
    }

    public Climber getDiagonalClimbers() {
        return diagonalClimber;
    }

    public boolean isAtTarget(boolean vertical) {
        if (vertical) {
            return verticalClimber.isAtTarget();
        } else {
            return diagonalClimber.isAtTarget();
        } 
    }

    public void climb(boolean vertical, double heightCM) {
        if (vertical) {
            verticalClimber.climb(heightCM);
        } else {
            diagonalClimber.climb(heightCM);
        }
    }
}

