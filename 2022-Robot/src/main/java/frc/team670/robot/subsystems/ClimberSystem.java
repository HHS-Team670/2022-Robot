package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
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
        climber1.stop();
        climber2.stop();
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

    public Climber getClimber1() {
        return climber1;
    }

    public Climber getClimber2() {
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

