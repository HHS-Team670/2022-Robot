package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

/**
 * 
 * @author Pallavi, Eugenia, Sofia, ctychen, Sanatan
 */
public class Climber extends MustangSubsystemBase {

    public TelescopingClimber straight;
    public TelescopingClimber oblique;

    public Climber() {
        straight = new TelescopingClimber(RobotMap.CLIMBER_MOTOR1, RobotMap.CLIMBER_MOTOR2);
        oblique = new TelescopingClimber(RobotMap.CLIMBER_MOTOR3, RobotMap.CLIMBER_MOTOR4);
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