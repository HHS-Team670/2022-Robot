/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.XboxRobotOrientedDrive;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.drivebase.TankDrive;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the 2022 drivebase.
 * It was originally an omni drivebase, but the hyperdrive (center module)
 * was disabled before the first competition because of technical issues.
 * The hyperdrive was removed altogether before the second competition,
 * and all references to the center drive were removed.
 * 
 * @author lakshbhambhani
 */
public class DriveBase extends TankDrive {

  private SparkMAXLite left1, left2, right1, right2;
  private RelativeEncoder left1Encoder, left2Encoder, right1Encoder, right2Encoder;

  private MustangController mController;

  private List<SparkMAXLite> leftControllers, rightControllers;
  private List<SparkMAXLite> allMotors = new ArrayList<SparkMAXLite>();;

  private NavX navXMicro;

  private DifferentialDriveOdometry odometry;

  private MustangCommand defaultCommand;

  SparkMaxPIDController leftPIDController;
  SparkMaxPIDController rightPIDController;

  public DriveBase(MustangController mustangController) {
    this.mController = mustangController;

    leftControllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SPARK_LEFT_MOTOR_1, RobotMap.SPARK_LEFT_MOTOR_2,
        false, MotorConfig.Motor_Type.NEO);
    rightControllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SPARK_RIGHT_MOTOR_1,
        RobotMap.SPARK_RIGHT_MOTOR_2, false, MotorConfig.Motor_Type.NEO);
    
    allMotors.addAll(leftControllers);
    allMotors.addAll(rightControllers);

    left1 = leftControllers.get(0);
    left2 = leftControllers.get(1);
    right1 = rightControllers.get(0);
    right2 = rightControllers.get(1);

    left1Encoder = left1.getEncoder();
    left2Encoder = left2.getEncoder();
    right1Encoder = right1.getEncoder();
    right2Encoder = right2.getEncoder();

    for(SparkMAXLite motorController : allMotors) {
      // Do NOT invert for the right side here
      motorController.getEncoder().setVelocityConversionFactor(RobotConstants.DRIVEBASE_VELOCITY_CONVERSION_FACTOR);
      motorController.getEncoder().setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);
    }

    // The DifferentialDrive inverts the right side automatically, however we want
    // invert straight from the Spark so that we can still use it properly with the
    // CANPIDController, so we need to tell differenetial drive to not invert.
    setMotorsInvert(leftControllers, false);
    setMotorsInvert(rightControllers, true); // Invert right controllers here so they will work properly with the CANPIDController

    super.setMotorControllers(new MotorController[] { left1, left2 }, new MotorController[] { right1, right2 },
        false, false, .1, true);

    // initialized NavX and sets Odometry. Position is zeroed.
    navXMicro = new NavX(RobotMap.NAVX_PORT);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), new Pose2d(0, 0, new Rotation2d()));

    initBrakeMode();

    leftPIDController = left1.getPIDController();
    rightPIDController = right1.getPIDController();

    leftPIDController.setP(RobotConstants.rightKPDriveVel);
    leftPIDController.setI(RobotConstants.rightKIDriveVel);
    leftPIDController.setD(RobotConstants.rightKDDriveVel);

    rightPIDController.setP(RobotConstants.rightKPDriveVel);
    rightPIDController.setI(RobotConstants.rightKIDriveVel);
    rightPIDController.setD(RobotConstants.rightKDDriveVel);

    leftPIDController.setOutputRange(-1, 1);
    rightPIDController.setOutputRange(-1, 1);

  }

  /**
   * Makes the DriveBase's default command initialize teleop
   */
  public void initDefaultCommand() {
    defaultCommand = new XboxRobotOrientedDrive(this, mController);
    MustangScheduler.getInstance().setDefaultCommand(this, defaultCommand);
  }

  public void cancelDefaultCommand() {
    MustangScheduler.getInstance().cancel(defaultCommand);
  }

  /**
   * Checks the health for driveBase. RED if all motors are dead, GREEN if all
   * motors are alive and navx is connected, YELLOW if a motor is disconnected or
   * navX is not connected
   */
  @Override
  public HealthState checkHealth() {
    return checkHealth(left1.isErrored(), left2.isErrored(), right1.isErrored(), right2.isErrored());
  }

  /**
   * Sets all motors to Brake Mode
   */
  public void initBrakeMode() {
    setMotorsNeutralMode(allMotors, IdleMode.kBrake);
  }

  /**
   * Sets all motors to Coast Mode
   */
  public void initCoastMode() {
    setMotorsNeutralMode(allMotors, IdleMode.kCoast);
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
   * Sets all motors in the specified list to be in the specified mode
   * @param motors Motors to be set to a particular IdleMode
   * @param mode The target mode (coast or brake)
   */
  public void setMotorsNeutralMode(List<SparkMAXLite> motors, IdleMode mode) {
    for (CANSparkMax m : motors) {
      m.setIdleMode(mode);
    }
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
   * Sets the ramp rate for the list of motors passed in.
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
    odometry.update(Rotation2d.fromDegrees(getHeading()), left1Encoder.getPosition(), right1Encoder.getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return current Pose2d, calculated by odometry
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose2d The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose2d) {
    // zeroHeading();
    navXMicro.reset(pose2d.getRotation().getDegrees() * (RobotConstants.kNavXReversed ? -1. : 1.));
    odometry.resetPosition(pose2d, pose2d.getRotation());

    //If encoders aren't being properly zeroed, check if lE and rE are REVLibError.kOk
    REVLibError lE = left1Encoder.setPosition(0);
    REVLibError rE = right1Encoder.setPosition(0);
  }

  /**
   * Resets the odometry to 0 and zeroes the encoders.
   */
  public void resetOdometry() {
    zeroHeading();
    odometry.resetPosition(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0));
    left1Encoder.setPosition(0);
    right1Encoder.setPosition(0);
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

  public void zeroEncoders() {
    left1Encoder.setPosition(0);
    right1Encoder.setPosition(0);
  }

  public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
    left1.setVoltage(leftVoltage);
    right1.setVoltage(rightVoltage);
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

  public SparkMaxPIDController getLeftSparkMaxPIDController(){
    return leftPIDController;
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

  public SparkMaxPIDController getRightSparkMaxPIDController(){
    return rightPIDController;
  }

  @Override
  public SimpleMotorFeedforward getRightSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(RobotConstants.rightKsVolts, RobotConstants.rightKvVoltSecondsPerMeter,
        RobotConstants.rightKaVoltSecondsSquaredPerMeter);
  }

  /**
   * Toggles the idle of each motor from either kBrake or kCoast to the other one.
   */
  @Override
  public void toggleIdleMode() {
    for (SparkMAXLite motor : allMotors) {
      if (motor.getIdleMode() == IdleMode.kBrake) {
        motor.setIdleMode(IdleMode.kCoast);
      } else {
        motor.setIdleMode(IdleMode.kBrake);
      }
    }
  }

  public void holdPosition() {
    getLeftSparkMaxPIDController().setReference(left1Encoder.getPosition(), CANSparkMax.ControlType.kPosition);
    getRightSparkMaxPIDController().setReference(right1Encoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public void releasePosition() {
    getLeftSparkMaxPIDController().setReference(0, CANSparkMax.ControlType.kDutyCycle);
    getRightSparkMaxPIDController().setReference(0, CANSparkMax.ControlType.kDutyCycle);
  }

  @Override
  public void debugSubsystem() {
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("currentX", getPose().getX());
    SmartDashboard.putNumber("currentY", getPose().getY());
    SmartDashboard.putNumber("left 1 encoder", getLeftPositionTicks());
    SmartDashboard.putNumber("right 1 encoder", getRightPositionTicks());
    SmartDashboard.putNumber("left velocity", left1Encoder.getVelocity());
    SmartDashboard.putNumber("right velocity", right1Encoder.getVelocity());
    SmartDashboard.putNumber("left position", left1Encoder.getPosition());
    SmartDashboard.putNumber("right position", right1Encoder.getPosition());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("pose X", getPose().getX());
    SmartDashboard.putNumber("pose Y", getPose().getY());
    sendEncoderDataToDashboard();
  }

}
