package frc.team670.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.constants.RobotMap;


public class ClimberContainer {


    private static final double MOTOR_MAX_RPM = 5676;

    //TODO: find FF values for vertical climbers
    private static final double VERTICAL_kFF = 1/MOTOR_MAX_RPM;
    private static final double VERTICAL_kP = 0.00001;
  


    private static final double SPOOL_DIAMETER_CM = 3.81;
    private static final double SPOOL_CIRCUMFERENCE_CM = SPOOL_DIAMETER_CM * Math.PI;

    private static final double VERTICAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 24;
    private static final double DIAGONAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION = 25;

    private static final double VERTICAL_ROTATIONS_PER_CM = VERTICAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION / SPOOL_CIRCUMFERENCE_CM;
    private static final double VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED = 0; // TODO find actual
    private static final double VERTICAL_CLIMBER_CM_TO_FULL_EXTENSION = 58.42;
    // private static final double VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = VERTICAL_ROTATIONS_PER_CM * VERTICAL_CLIMBER_CM_TO_FULL_EXTENSION;
    private static final double VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = 108;

   // SmartMotion constants 
    // TODO: different for diagonal vs vertical?
    private static final double VERTICAL_MAX_ACC = 5676; // TODO test
    private static final double VERTICAL_MIN_VEL = 0; // TODO test
    private static final double VERTICAL_MAX_VEL = 5676; // TODO test

    //TODO: find FF values for diagonal climbers
    private static final double DIAGONAL_kFF = 1/MOTOR_MAX_RPM;
    private static final double DIAGONAL_kP = 0.00001;
    
    private static final double DIAGONAL_ROTATIONS_PER_CM = DIAGONAL_MOTOR_ROTATIONS_PER_SPOOL_ROTATION / SPOOL_CIRCUMFERENCE_CM;
    private static final double DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED = 0;
    private static final double DIAGONAL_CLIMBER_CM_TO_FULL_EXTENSION = 79.2325;
    private static final double DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION = DIAGONAL_ROTATIONS_PER_CM * DIAGONAL_CLIMBER_CM_TO_FULL_EXTENSION; 
    
    private static final double DIAGONAL_MAX_ACC = 200; // TODO test
    private static final double DIAGONAL_MIN_VEL = 0; // TODO test
    private static final double DIAGONAL_MAX_VEL = 200; // TODO test

    public static ArrayList<Climber> getClimbers() {

        ArrayList<Climber> climbers = new ArrayList<Climber>();

        Climber verticalClimber = new Climber(RobotMap.VERTICAL_CLIMBER, VERTICAL_kFF, VERTICAL_kP,
                VERTICAL_MOTOR_ROTATIONS_AT_RETRACTED, VERTICAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, VERTICAL_ROTATIONS_PER_CM,
                VERTICAL_MAX_ACC, VERTICAL_MIN_VEL, VERTICAL_MAX_VEL);   
        
        Climber diagonalClimber = new Climber(RobotMap.DIAGONAL_CLIMBER, DIAGONAL_kFF, DIAGONAL_kP,
                DIAGONAL_MOTOR_ROTATIONS_AT_RETRACTED, DIAGONAL_MOTOR_ROTATIONS_AT_MAX_EXTENSION, DIAGONAL_ROTATIONS_PER_CM,
                DIAGONAL_MAX_ACC, DIAGONAL_MIN_VEL, DIAGONAL_MAX_VEL);
        
        climbers.add(verticalClimber);
        climbers.add(diagonalClimber);

        return climbers;

            
    }

    public static Climber getVerticalClimber(){
        return getClimbers().get(0);
    }

    public static Climber getDiagonalClimber(){
        return getClimbers().get(1);
    }
}

