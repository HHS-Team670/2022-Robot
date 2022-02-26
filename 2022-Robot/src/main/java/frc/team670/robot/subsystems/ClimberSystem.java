package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static final double VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final double VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = VERTICAL_ROTATIONS_PER_CM * 58.42;


    //TODO: find PIDF values for diagonal climbers
    private static final double DIAGONAL_kP = 0;
    private static final double DIAGONAL_kI = 0;
    private static final double DIAGONAL_kD = 0;
    private static final double DIAGONAL_kFF = 0;
    
    private static final double DIAGONAL_ROTATIONS_PER_CM = 2.0886;
    private static final double DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final double DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = DIAGONAL_ROTATIONS_PER_CM * 79.2325; 

    private Climber verticalClimber;
    private Climber diagonalClimber;

    public ClimberSystem() {

            verticalClimber = new Climber(RobotMap.VERTICAL_CLIMBER, RobotMap.VERTICAL_CLIMBER_LIMIT_SWITCH, VERTICAL_kP, VERTICAL_kI, VERTICAL_kD, VERTICAL_kFF,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, VERTICAL_ROTATIONS_PER_CM);   
        
            diagonalClimber = new Climber(RobotMap.DIAGONAL_CLIMBER, RobotMap.DIAGONAL_CLIMBER_LIMIT_SWITCH, DIAGONAL_kP, DIAGONAL_kI, DIAGONAL_kD, DIAGONAL_kFF,
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
        SmartDashboard.putNumber("Vertical Climber Motor Rotations difference",  VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION - verticalClimber.getUnadjustedMotorRotations());
        SmartDashboard.putNumber("Diagonal Climber Motor Rotations difference", DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION - diagonalClimber.getUnadjustedMotorRotations());
    }

    @Override
    public void debugSubsystem() {
    }

    public Climber getVerticalClimber() {
        return verticalClimber;
    }

    public Climber getDiagonalClimber() {
        return diagonalClimber;
    }
}

