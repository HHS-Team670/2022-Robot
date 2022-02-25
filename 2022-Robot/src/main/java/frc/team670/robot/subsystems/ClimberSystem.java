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

    private static final float VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0; //TODO: talk to mech to find max motor rotations for full extension

    //TODO: find PIDF values for diagonal climbers
    private static final double DIAGONAL_kP = 0;
    private static final double DIAGONAL_kI = 0;
    private static final double DIAGONAL_kD = 0;
    private static final double DIAGONAL_kFF = 0;

    private static final float DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0; //TODO: talk to mech to find max motor rotations for full extension

    private ArrayList<Climber> verticalClimbers;
    private ArrayList<Climber> diagonalClimbers;

    public ClimberSystem() {
        verticalClimbers.add( 
            new Climber(RobotMap.VERTICAL_CLIMBER_1, VERTICAL_kP, VERTICAL_kI, VERTICAL_kD, VERTICAL_kFF,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, 66.24));
        verticalClimbers.add(
            new Climber(RobotMap.VERTICAL_CLIMBER_2, VERTICAL_kP, VERTICAL_kI, VERTICAL_kD, VERTICAL_kFF,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, 66.24));    
        
                
        diagonalClimbers.add(
            new Climber(RobotMap.DIAGONAL_CLIMBER_1, DIAGONAL_kP, DIAGONAL_kI, DIAGONAL_kD, DIAGONAL_kFF,
                DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED, DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, 66.24));
        diagonalClimbers.add(
            new Climber(RobotMap.DIAGONAL_CLIMBER_2, DIAGONAL_kP, DIAGONAL_kI, DIAGONAL_kD, DIAGONAL_kFF,
                DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED, DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, 66.24));
    }

    public void stop() {
        for (Climber verticalClimber : verticalClimbers) {
            verticalClimber.stop();
        }
        for (Climber diagonalClimber : diagonalClimbers) {
            diagonalClimber.stop();
        }
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        if (verticalClimbers.get(0).checkHealth() == HealthState.GREEN && 
        verticalClimbers.get(1).checkHealth() == HealthState.GREEN && diagonalClimbers.get(0).checkHealth() == HealthState.GREEN
        && diagonalClimbers.get(1).checkHealth() == HealthState.GREEN) {
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

    public ArrayList<Climber> getVerticalClimbers() {
        return verticalClimbers;
    }

    public ArrayList<Climber> getDiagonalClimbers() {
        return diagonalClimbers;
    }

    public boolean isAtTarget(boolean vertical) {
        if (vertical) {
            boolean bool = true;

            for (Climber c : verticalClimbers) {
                bool = bool && c.isAtTarget(); //if any climber is not at target, then bool will be false
            }
            return bool;

        } else {
            boolean bool = true;

            for (Climber c : diagonalClimbers) {
                bool = bool && c.isAtTarget(); //if any climber is not at target, then bool will be false
            }
            return bool;
        } 
    }

    public void climb(boolean vertical, double heightCM) {
        if (vertical) {
            for (Climber c : verticalClimbers) {
                c.climb(heightCM);
            }
        } else {
            for (Climber c : diagonalClimbers) {
                c.climb(heightCM);
            }
        }
    }
}

