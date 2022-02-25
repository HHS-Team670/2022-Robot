package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap {

  public static final int PDP_ID = 0;

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  // Drive Base
  public static final int SPARK_LEFT_MOTOR_1 = 15; // These are properly set.
  public static final int SPARK_LEFT_MOTOR_2 = 14;
  public static final int SPARK_RIGHT_MOTOR_1 = 12;
  public static final int SPARK_RIGHT_MOTOR_2 = 10;
  public static final int SPARK_MIDDLE_MOTOR = 13;
  public final static SerialPort.Port NAVX_PORT = SerialPort.Port.kUSB;

  // Shooter
  public final static int SHOOTER_MAIN = 5;
  public final static int SHOOTER_FOLLOWER = 4;
  public static final int SHOOTER_ULTRASONIC_TPIN = 6;
  public static final int SHOOTER_ULTRASONIC_EPIN = 7;

  // Conveyor
  public static final int INTAKE_CONVEYOR_MOTOR = 7;
  public static final int SHOOTER_CONVEYOR_MOTOR = 6;

  public static final int INTAKE_CONVEYOR_BEAMBREAK = 1;
  public static final int SHOOTER_CONVEYOR_BEAMBREAK = 0;

  // Climber
  public static final int VERTICAL_CLIMBER = 3; // TODO get id
  public static final int DIAGONAL_CLIMBER = 2; // TODO get id

  // Intake
  public static final int INTAKE_ROLLER = 11;
  public static final int FLIP_OUT = 9;
  public static final int FLIP_OUT_ABS_ENCODER = 2;

  // leds
  public static final int LED_PORT = 9;

}