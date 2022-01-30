// COPIED FROM 2020


package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.SerialPort;
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

  // NavX
  public final static I2C.Port NAVX_PORT = I2C.Port.kOnboard;
  public final static SerialPort.Port ARDUINO_PORT = SerialPort.Port.kUSB2;

  // Shooter
  public final static int SHOOTER_MAIN = 11;
  public final static int SHOOTER_FOLLOWER = 12;

  // Turret
  public static final int TURRET_ROTATOR = 24;

  // Color Wheel
  public static final int COLOR_WHEEL_MOTOR_ID = 7;

  // Intake
  public static final int INTAKE_ROLLER = 4;

  // Conveyor
  public static final int CONVEYOR_ROLLER = 8;

  // Indexer and Updraw
  public static final int FRONT_MOTOR = 25;
  public static final int BACK_MOTOR = 26;
  public static final int UPDRAW_SPINNER = 10;

  // Climber
  public static final int CLIMBER_MOTOR = 14;

  // PNEUMATICS
  public static final int PCMODULE = 2; // PCM CAN ID

  // Solenoids. Using 24V

  public static final int INTAKE_DEPLOYER = 0; 
  public static final int INDEXER_PUSHER_CLIMBER_DEPLOY = 1;
  public static final int VISION_LED_PCM = 4;

  // SENSORS

  // Indexer sensors
  public static final I2C.Port INDEXER_MUL_PORT = I2C.Port.kOnboard;


  // LEDs (connected on RoboRIO PWM)
  public static final int LEFT_SIDE_LEDS_PWM = 0;
  public static final int RIGHT_SIDE_LEDS_PWM = 1;

  public static final int INTAKE_CONVEYOR_MOTOR = -1;//Needs to be changed
  public static final int SHOOTER_CONVEYOR_MOTOR = -1;//Needs to be changed

  public static final int INTAKE_CONVEYOR_BEAMBREAK = -1;//Needs to be changed
  public static final int SHOOTER_CONVEYOR_BEAMBREAK = -1;//Needs to be changed

}
