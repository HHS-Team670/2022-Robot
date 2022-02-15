package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap {

  public static final int PDP_ID = 0;

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  // Drive Base
  public static final int SPARK_LEFT_MOTOR_1 = 15; // These are properly set.
  public static final int SPARK_LEFT_MOTOR_2 = 12;
  public static final int SPARK_RIGHT_MOTOR_1 = 14;
  public static final int SPARK_RIGHT_MOTOR_2 = 10;
  public static final int SPARK_MIDDLE_MOTOR = 13;

  // NavX
  public final static SerialPort.Port NAVX_PORT = SerialPort.Port.kUSB;
  public final static SerialPort.Port ARDUINO_PORT = SerialPort.Port.kUSB2;

  // Shooter
  public final static int SHOOTER_MAIN = 4;
  public final static int SHOOTER_FOLLOWER = 5;

  public final static int INTAKE_ROLLER = 11;

  //CONVEYOR Vars
  public static final int INTAKE_CONVEYOR_MOTOR = 7;
  public static final int SHOOTER_CONVEYOR_MOTOR = 6;

  public static final int INTAKE_CONVEYOR_BEAMBREAK = 1;
  public static final int SHOOTER_CONVEYOR_BEAMBREAK = 0;
  

  // Intake
  public static final int INTAKE_ROLLER = 0;
  public static final int DEPLOYER_MOTOR = 0;

  public static final int SHOOTER_ULTRASONIC_TPIN = 6;
  public static final int SHOOTER_ULTRASONIC_EPIN = 7;

}