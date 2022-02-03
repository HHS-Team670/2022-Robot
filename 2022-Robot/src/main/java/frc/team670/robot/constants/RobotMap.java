package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.I2C;

public class RobotMap {

  public static final int PDP_ID = 0;

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  // Drive Base
  public static final int SPARK_LEFT_MOTOR_1 = 20; // These are properly set.
  public static final int SPARK_LEFT_MOTOR_2 = 21;
  public static final int SPARK_RIGHT_MOTOR_1 = 22;
  public static final int SPARK_RIGHT_MOTOR_2 = 23;
  public static final int SPARK_MIDDLE_MOTOR = 24;

  // NavX
  public final static I2C.Port NAVX_PORT = I2C.Port.kOnboard;

  // Shooter
  public final static int SHOOTER_MAIN = 5;
  public final static int SHOOTER_FOLLOWER = 4;

  //CONVEYOR Vars
  public static final int INTAKE_CONVEYOR_MOTOR = 7;
  public static final int SHOOTER_CONVEYOR_MOTOR = 6;

  public static final int INTAKE_CONVEYOR_BEAMBREAK = 1;
  public static final int SHOOTER_CONVEYOR_BEAMBREAK = 0;

}
