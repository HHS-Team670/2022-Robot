package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.constants.RobotMap;


public class ClimberSystem extends MustangSubsystemBase {


    private static final double MOTOR_MAX_RPM = 5676;

    //TODO: find FF values for vertical climbers
    private static final double VERTICAL_kFF = 1/MOTOR_MAX_RPM;
    

    private static final double SPOOL_DIAMETER_CM = 3.81;
    private static final double SPOOL_CIRCUMFERENCE_CM = SPOOL_DIAMETER_CM * Math.PI;

    private static final double VERTICAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 24;
    private static final double DIAGONAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 25;

    private static final double VERTICAL_ROTATIONS_PER_CM = VERTICAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION / SPOOL_CIRCUMFERENCE_CM;
    private static final double VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final double VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = VERTICAL_ROTATIONS_PER_CM * 58.42;

   // SmartMotion constants 
    // TODO: different for diagonal vs vertical?
    private final double VERTICAL_MAX_ACC = 200; // TODO test
    private final double VERTICAL_MIN_VEL = 0; // TODO test
    private final double VERTICAL_MAX_VEL = 200; // TODO test

    //TODO: find FF values for diagonal climbers
    private static final double DIAGONAL_kFF = 1/MOTOR_MAX_RPM;
    
    private static final double DIAGONAL_ROTATIONS_PER_CM = DIAGONAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION / SPOOL_CIRCUMFERENCE_CM;
    private static final double DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final double DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = DIAGONAL_ROTATIONS_PER_CM * 79.2325; 
    
    private final double DIAGONAL_MAX_ACC = 200; // TODO test
    private final double DIAGONAL_MIN_VEL = 0; // TODO test
    private final double DIAGONAL_MAX_VEL = 200; // TODO test

    private Climber verticalClimber;
    private Climber diagonalClimber;

    public ClimberSystem() {

            verticalClimber = new Climber(RobotMap.VERTICAL_CLIMBER, VERTICAL_kFF,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, VERTICAL_ROTATIONS_PER_CM,
                VERTICAL_MAX_ACC, VERTICAL_MIN_VEL, VERTICAL_MAX_VEL);   
        
            diagonalClimber = new Climber(RobotMap.DIAGONAL_CLIMBER, DIAGONAL_kFF,
                DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED, DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, DIAGONAL_ROTATIONS_PER_CM,
                DIAGONAL_MAX_ACC, DIAGONAL_MIN_VEL, DIAGONAL_MAX_VEL);
    }

    public void stop() {
        verticalClimber.stop();
        diagonalClimber.stop();
    }

    @Override
    public HealthState checkHealth() {
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

