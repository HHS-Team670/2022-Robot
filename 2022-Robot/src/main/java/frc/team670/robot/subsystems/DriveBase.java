/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// COPIED FROM 2020

package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.XboxRocketLeagueDrive;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.drivebase.TankDriveBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents a tank drive base.
 * 
 * @author lakshbhambhani
 */
public class DriveBase extends TankDriveBase {
  private SparkMAXLite left1, left2, right1, right2;
  private RelativeEncoder left1Encoder, left2Encoder, right1Encoder, right2Encoder;

  private MustangController mController;

  private List<SparkMAXLite> leftControllers, rightControllers;
  private List<SparkMAXLite> allMotors = new ArrayList<SparkMAXLite>();;

  private NavX navXMicro;

  private DifferentialDrivePoseEstimator poseEstimator;

  private static final double CURRENT_WHEN_AGAINST_BAR = 5; // TODO Find this
  private int againstBarCount = 0;

  public static final double START_Y = 2.4;
  public static final double START_X = 15.983 - 3.8;
  public static final double START_ANGLE_DEG = 180;
  public static final Rotation2d START_ANGLE_RAD = Rotation2d.fromDegrees(START_ANGLE_DEG);


  // public static final double 

  // Constants used for doing robot to target pose conversion

  public static final Pose2d TARGET_POSE = new Pose2d(15.983, 2.4, Rotation2d.fromDegrees(0));

  public static final Pose2d CAMERA_OFFSET = 
    TARGET_POSE.transformBy(new Transform2d(new Translation2d(-0.23, 0), Rotation2d.fromDegrees(0)));
public DriveBase(MustangController mustangController) {
  leftControllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SPARK_LEFT_MOTOR_1, RobotMap.SPARK_LEFT_MOTOR_2,
  false, MotorConfig.Motor_Type.NEO);
rightControllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SPARK_RIGHT_MOTOR_1,
  RobotMap.SPARK_RIGHT_MOTOR_2, false, MotorConfig.Motor_Type.NEO);

left1 = leftControllers.get(0);
left2 = leftControllers.get(1);
right1 = rightControllers.get(0);
right2 = rightControllers.get(1);

left1Encoder = left1.getEncoder();
right1Encoder = right1.getEncoder();
left2Encoder = left2.getEncoder();
right2Encoder = right2.getEncoder();

left1Encoder.setVelocityConversionFactor(RobotConstants.sparkMaxVelocityConversionFactor);
left2Encoder.setVelocityConversionFactor(RobotConstants.sparkMaxVelocityConversionFactor); // Do not invert for
                                                                                         // right side
right1Encoder.setVelocityConversionFactor(RobotConstants.sparkMaxVelocityConversionFactor);
right2Encoder.setVelocityConversionFactor(RobotConstants.sparkMaxVelocityConversionFactor);

left1Encoder.setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);
left2Encoder.setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);
right1Encoder.setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);
right2Encoder.setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);

allMotors.addAll(leftControllers);
allMotors.addAll(rightControllers);

// The DifferentialDrive inverts the right side automatically, however we want
// invert straight
// from the Spark so that we can still use it properly with the
// CANPIDController, so we need to tell
// differenetial drive to not invert.
setMotorsInvert(leftControllers, false);
setMotorsInvert(rightControllers, true); // Invert this so it will work properly with the CANPIDController

super.setMotorControllers(new MotorController[] { left1, left2 }, new MotorController[] { right1, right2 }, false,
  false, .1, true);

// initialized NavX and sets Odometry
navXMicro = new NavX(RobotMap.NAVX_PORT);
// AHRS navXMicro = new AHRS(RobotMap.NAVX_PORT);
    
  }

  /**
   * Used to initialized teleop command for the driveBase
   */
  public void initDefaultCommand() {
    MustangScheduler.getInstance().setDefaultCommand(this, new XboxRocketLeagueDrive(this, mController));
  }

  /**
   * Checks the health for driveBase. RED if all motors are dead, GREEN if all
   * motors are alive and navx is connected, YELLOW if a motor is disconnected or
   * nav is not connected
   */
  @Override
  public HealthState checkHealth() {
    HealthState state = HealthState.GREEN;

    
    return state;
  }

  /**
   * Sets all motors to Brake Mode
   */
  public void initBrakeMode() {
    setMotorsBrakeMode(allMotors, IdleMode.kBrake);
  }

  /**
   * Sets all motors to Coast Mode
   */
  public void initCoastMode() {
    setMotorsNeutralMode(IdleMode.kCoast);
  }

  /*
   * Gets the input voltage of all the motor controllers on the robot
   */
  public double getRobotInputVoltage() {
    double output = left1.getBusVoltage() + left2.getBusVoltage() + right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of all the motor controllers on the robot
   */
  public double getRobotOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput() + right1.getAppliedOutput()
        + right2.getAppliedOutput();
    return output;
  }

  /**
   * Inverts a list of motors.
   */
  private void setMotorsInvert(List<SparkMAXLite> motorGroup, boolean invert) {
    for (CANSparkMax m : motorGroup) {
      m.setInverted(invert);
    }
  }

  /**
   * Sets array of motors to be of a specified mode
   */
  public void setMotorsNeutralMode(IdleMode mode) {
    for (CANSparkMax m : allMotors) {
      m.setIdleMode(mode);
    }
  }

  /**
   * Sets array of motor to coast mode
   */
  public void setMotorsCoastMode(List<CANSparkMax> motorGroup, IdleMode mode) {
    for (CANSparkMax m : motorGroup) {
      m.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Sets array of motor to brake mode
   */
  public void setMotorsBrakeMode(List<SparkMAXLite> motorGroup, IdleMode mode) {
    for (CANSparkMax m : motorGroup) {
      m.setIdleMode(IdleMode.kBrake);
    }
  }

  /*
   * Gets the voltage fed into the motor controllers on the left side of the robot
   */
  public double getLeftInputVoltage() {
    double output = left1.getBusVoltage() + left2.getBusVoltage();
    return output;
  }

  /*
   * Get the voltage fed into the motor controllers on the right side of the robot
   */
  public double getRightInputVoltage() {
    double output = right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of the motor controllers on the left side of the
   * robot
   */
  public double getLeftOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output voltage of the motor controllers on the right side of the
   * robot
   */
  public double getRightOutputVoltage() {
    double output = right1.getAppliedOutput() + right2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output current (in amps) of the motor controllers on the left side
   * of the robot
   */
  public double getLeftOutputCurrent() {
    double output = left1.getOutputCurrent() + left2.getOutputCurrent();
    return output;
  }

  /*
   * Gets the output current (in amps) of the motor controllers on the right side
   * of the robot
   */
  public double getRightOutputCurrent() {
    double output = right1.getOutputCurrent() + right2.getOutputCurrent();
    return output;
  }

  /*
   * Gets the output current of all the motor controllers on the robot
   */
  public double getRobotOutputCurrent() {
    double output = left1.getOutputCurrent() + left2.getOutputCurrent() + right1.getOutputCurrent()
        + right2.getOutputCurrent();
    return output;
  }

  /**
   * Returns the Spark Max Encoder for the Left Main Motor
   */
  public RelativeEncoder getLeftMainEncoder() {
    return left1Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Left Follower Motor
   */
  public RelativeEncoder getLeftFollowerEncoder() {
    return left2Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Right Main Motor
   */
  public RelativeEncoder getRightMainEncoder() {
    return right1Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Right Follower Motor
   */
  public RelativeEncoder getRightFollowerEncoder() {
    return right2Encoder;
  }

  /**
   * Returns the Left Motor Controllers
   * 
   * @return The list of the motor controllers on the left side of the robot
   */
  public List<SparkMAXLite> getLeftControllers() {
    return leftControllers;
  }

  /**
   * Returns the Right Motor Controller
   * 
   * @return The list of the motor controllers on the right side of the robot
   */
  public List<SparkMAXLite> getRightControllers() {
    return rightControllers;
  }

  /**
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setRampRate(List<SparkMAXLite> motors, double rampRate) {
    for (CANSparkMax m : motors) {
      m.setClosedLoopRampRate(rampRate);
      m.setOpenLoopRampRate(rampRate);
    }
  }

  /**
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setTeleopRampRate() {
    setRampRate(allMotors, 0.36); // Will automatically cook some Cheezy Poofs
  }

  public void sendEncoderDataToDashboard() {
    SmartDashboard.putNumber("Left M Position Ticks", left1Encoder.getPosition());
    SmartDashboard.putNumber("Left M Velocity Ticks", left1Encoder.getVelocity());
    SmartDashboard.putNumber("Left S Position Ticks", left2Encoder.getPosition());
    SmartDashboard.putNumber("Left S Velocity Ticks", left2Encoder.getVelocity());
    SmartDashboard.putNumber("Right M Position Ticks", right1Encoder.getPosition());
    SmartDashboard.putNumber("Right M Velocity Ticks", right1Encoder.getVelocity());
    SmartDashboard.putNumber("Right S Position Ticks", right2Encoder.getPosition());
    SmartDashboard.putNumber("Right S Velocity Ticks", right2Encoder.getVelocity());
  }

  @Override
  public void mustangPeriodic() {
    

  }

  

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    zeroHeading();
    poseEstimator.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    REVLibError lE = left1Encoder.setPosition(0);
    REVLibError rE = right1Encoder.setPosition(0);
    SmartDashboard.putString("Encoder return value left", lE.toString());
    SmartDashboard.putString("Encoder return value right", rE.toString());
    SmartDashboard.putNumber("Encoder positions left", left1Encoder.getPosition()); 
    SmartDashboard.putNumber("Encoder positions left", right1Encoder.getPosition()); 
    int counter = 0;
    while ((left1Encoder.getPosition() != 0 || right1Encoder.getPosition() != 0) && counter < 30) {
      lE = left1Encoder.setPosition(0);
      rE = right1Encoder.setPosition(0);
      counter++;
    }
    // Logger.consoleLog("Drivebase pose reset %s", pose);
    // Logger.consoleLog("Drivebase get position after reset %s %s",
    // left1Encoder.getPosition(), right1Encoder.getPosition());
  }

  public void resetOdometry() {
    zeroHeading();
    left1Encoder.setPosition(0);
    right1Encoder.setPosition(0);
    poseEstimator = new DifferentialDrivePoseEstimator(Rotation2d.fromDegrees(getHeading()),
        new Pose2d(0, 0, new Rotation2d()), VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01),
        VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)), // TODO: find correct values
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // TODO: find correct values

  }

  /**
   * Returns the heading of the robot.
   * 
   * @return the robot's heading in degrees, in range [-180, 180]
   */
  public double getHeading() {
    return Math.IEEEremainder(navXMicro.getAngle(), 360) * (RobotConstants.kNavXReversed ? -1. : 1.);
  }

  /**
   * Returns the wheel speeds of the leftMain Motor and rightMainMotor
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left1Encoder.getVelocity(), right1Encoder.getVelocity());
  }

  public void zeroSensors() {
    left1Encoder.setPosition(0);
    right1Encoder.setPosition(0);
  }

  public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
    left1.setVoltage(leftVoltage);
    right1.setVoltage(rightVoltage);
    getDriveTrain().feed();
  }

  public boolean isAlignedOnFloorBars() {
    double backLeftCurrent = left2.getOutputCurrent();
    double backRightCurrent = right2.getOutputCurrent();
    if (backLeftCurrent > 0.2 && backRightCurrent > 0.2) {
      if (backLeftCurrent >= CURRENT_WHEN_AGAINST_BAR && backRightCurrent >= CURRENT_WHEN_AGAINST_BAR) {
        againstBarCount++;
      } else {
        againstBarCount = 0;
      }
      if (againstBarCount >= 4) { // 4 consecutive readings higher than peak
        return true;
      }
    }
    return false;
  }

  @Override
  public double getLeftPositionTicks() {
    return (int) (left1Encoder.getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
  }

  @Override
  public double getLeftVelocityTicks() {
    return (left1Encoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
  }

  @Override
  public MustangController getMustangController() {
    return mController;
  }

  @Override
  public double getRightPositionTicks() {
    return (int) (left1Encoder.getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
  }

  @Override
  public double getRightVelocityTicks() {
    return (right1Encoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
  }

  @Override
  public boolean isQuickTurnPressed() {
    return mController.getRightBumper();
  }

  @Override
  public void setEncodersPositionControl(double leftPos, double rightPos) {
    left1Encoder.setPosition(leftPos);
    right1Encoder.setPosition(rightPos);
  }

  @Override
  public void setRampRate(double rampRate) {
    left1.setOpenLoopRampRate(rampRate);
    right1.setOpenLoopRampRate(rampRate);
  }

  @Override
  public void setVelocityControl(double leftSpeed, double rightSpeed) {
    left1.set(leftSpeed);
    right1.set(rightSpeed);
  }

  /**
   * Returns the velocity of the right side of the drivebase in inches/second from
   * the Spark Encoder
   */
  @Override
  public double ticksToInches(double ticks) {
    double rotations = ticks / RobotConstants.SPARK_TICKS_PER_ROTATION;
    return rotations * Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER;
  }

  @Override
  public double inchesToTicks(double inches) {
    double rotations = inches / (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
    return rotations * RobotConstants.SPARK_TICKS_PER_ROTATION;
  }

  @Override
  public void zeroHeading() {
    navXMicro.reset();
  }

  @Override
  public DifferentialDriveKinematics getKDriveKinematics() {
    return RobotConstants.kDriveKinematics;
  }

  @Override
  public PIDController getLeftPIDController() {
    return new PIDController(RobotConstants.leftKPDriveVel, RobotConstants.leftKIDriveVel,
        RobotConstants.leftKDDriveVel);
  }

  @Override
  public SimpleMotorFeedforward getLeftSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(RobotConstants.leftKsVolts, RobotConstants.leftKvVoltSecondsPerMeter,
        RobotConstants.leftKaVoltSecondsSquaredPerMeter);
  }

  @Override
  public PIDController getRightPIDController() {
    return new PIDController(RobotConstants.rightKPDriveVel, RobotConstants.rightKIDriveVel,
        RobotConstants.rightKDDriveVel);
  }

  @Override
  public SimpleMotorFeedforward getRightSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(RobotConstants.rightKsVolts, RobotConstants.rightKvVoltSecondsPerMeter,
        RobotConstants.rightKaVoltSecondsSquaredPerMeter);
  }

  @Override
  public void resetOdometry(frc.team670.mustanglib.subsystems.drivebase.Pose2d pose) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void resetOdometry(frc.team670.mustanglib.subsystems.drivebase.Pose2d pose) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void resetOdometry(frc.team670.mustanglib.subsystems.drivebase.Pose2d pose) {
    // TODO Auto-generated method stub
    
  }

}