package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

/**
 * 
 * @author Pallavi, Eugenia, Sofia, ctychen, Sanatan
 */
public class Climber extends MustangSubsystemBase {

    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kFF = 0;

    private static final float MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final float MOTOR_ROTATIONS_AT_MAX_EXTENSION = 0;

    private static final double kPO = 0;
    private static final double kIO = 0;
    private static final double kDO = 0;
    private static final double kFFO = 0;

    private static final float MOTOR_ROTATIONS_AT_RETRACTEDO = 0;
    private static final float MOTOR_ROTATIONS_AT_MAX_EXTENSIONO = 0;

    public TelescopingClimber straight;
    public TelescopingClimber oblique;

    public Climber() {
        straight = new TelescopingClimber(RobotMap.CLIMBER_MOTOR1, RobotMap.CLIMBER_MOTOR2, new double[]{kP, kI, kD, kFF}, new float[]{MOTOR_ROTATIONS_AT_RETRACTED, MOTOR_ROTATIONS_AT_MAX_EXTENSION}, 66.24);
        oblique = new TelescopingClimber(RobotMap.CLIMBER_MOTOR3, RobotMap.CLIMBER_MOTOR4, new double[]{kPO, kIO, kDO, kFFO}, new float[]{MOTOR_ROTATIONS_AT_RETRACTEDO, MOTOR_ROTATIONS_AT_MAX_EXTENSIONO}, 66.24);
    }

    public void stop()
    {
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
        System.out.println("Sanatan Was Here");
    }
}