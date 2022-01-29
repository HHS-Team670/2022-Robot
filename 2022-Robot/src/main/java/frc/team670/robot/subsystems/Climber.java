package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

/**
 * 
 * @author Pallavi, Eugenia, Sofia, ctychen, Sanatan
 */
public class Climber extends MustangSubsystemBase {

    private static final double STRAIGHT_kP = 0;
    private static final double STRAIGHT_kI = 0;
    private static final double STRAIGHT_kD = 0;
    private static final double STRAIGHT_kFF = 0;

    private static final float STRAIGHT_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float STRAIGHT_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0;

    private static final double OBLIQUE_kP = 0;
    private static final double OBLIQUE_kI = 0;
    private static final double OBLIQUE_kD = 0;
    private static final double OBLIQUE_kFF = 0;

    private static final float OBLIQUE_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float OBLIQUE_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0;

    public TelescopingClimber straight;
    public TelescopingClimber oblique;

    public Climber() {
        straight = new TelescopingClimber(RobotMap.STRAIGHT_CLIMBER_MOTOR, STRAIGHT_kP, STRAIGHT_kI, STRAIGHT_kD, STRAIGHT_kFF, STRAIGHT_MOTOR_ROTATIONS_AT_RETRACTED, STRAIGHT_MOTOR_ROTATIONS_AT_MAX_EXTENSION, 66.24);
        oblique = new TelescopingClimber(RobotMap.OBLIQUE_CLIMBER_MOTOR, OBLIQUE_kP, OBLIQUE_kI, OBLIQUE_kD, OBLIQUE_kFF, OBLIQUE_MOTOR_ROTATIONS_AT_RETRACTED, OBLIQUE_MOTOR_ROTATIONS_AT_MAX_EXTENSION, 66.24);
    }

    public void stop() {
        straight.setPower(0.0);
        oblique.setPower(0.0);
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return straight.checkHealth();
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub
    }
}