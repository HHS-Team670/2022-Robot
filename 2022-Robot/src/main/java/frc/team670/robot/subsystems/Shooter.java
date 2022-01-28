
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import frc.team670.robot.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.math.interpolable.InterpolatingDouble;
import frc.team670.mustanglib.utils.math.interpolable.InterpolatingTreeMap;
import frc.team670.mustanglib.utils.math.interpolable.LinearRegression;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.commands.shooter.SetRPMTarget;
import frc.team670.robot.constants.RobotConstants;


/**
 * Represents a 2-stage shooter, with 1st stage using a VictorSPX and 2-NEO 2nd
 * stage with SparkMax controllers.
 * 
 * @author 
 */
public class Shooter extends MustangSubsystemBase {

  private SparkMAXLite mainController, followerController;
  private List<SparkMAXLite> controllers;

  private RelativeEncoder shooter_mainEncoder;
  private SparkMaxPIDController shooter_mainPIDController;

  private double targetRPM = 0; // Will change later if we adjust by distance
  private static double DEFAULT_SPEED = 0;

  private static double MIN_RPM = 0;
  private static double MAX_RPM = 2725;

  private double speedAdjust = 0; // By default, we don't adjust, but this may get set later

  private static double MAX_SHOT_DISTANCE_METERS = 8.6868; // = 28-29ish feet

  private static final double PULLEY_RATIO = 2; // Need to check this

  private boolean ballHasBeenShot;
  private int shootingCurrentCount = 0;

  private static final double NORMAL_CURRENT = 0; // TODO: unknown

  private static final double V_P = 0.000100;
  private static final double V_I = 0.0;
  private static final double V_D = 0.0;
  private static final double V_FF = 0.000183;
  private static final double RAMP_RATE = 1.0;

  private static Vision vision;

  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> FLYWHEEL_RPM_MAP = new InterpolatingTreeMap<>();

  // The following is for using fancy interpolation based on 254's 2017 code, where we sample 
  // a ton and build a polynomial regression.

  // Format: {Distance from target in meters, RPM}
  // Distance currently from bumper
  private static final double[][] FLYWHEEL_RPM_AT_DISTANCE = { 

    { 0, 0},  // 10.9 ft  2125 rpm 
    { 0, 0 }, // 15 ft  2275 rpm 
    { 0, 0 }, // 24 ft 2575 rpm 
    { 0, 0 } // 28.5 ft 2725 rpm 

  };

  static {
    for (double[] pair : FLYWHEEL_RPM_AT_DISTANCE) {
      FLYWHEEL_RPM_MAP.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
    DEFAULT_SPEED = FLYWHEEL_RPM_MAP.getInterpolated(new InterpolatingDouble(MAX_SHOT_DISTANCE_METERS)).value;
  }

  // For now this should be enough: a linear regression for the relationship between distance
  // and RPM from this data should have a correlation coefficient of 0.9999 so it should be fine

   private static final double[] measuredDistancesMeters = {
     0,  // 10.9 ft  2125 rpm 
     0, // 15 ft  2275 rpm 
     0, // 24 ft 2575 rpm
     0, 
     0,// trench (28-29ft)
  };

   private static final double[] measuredRPMs = {
     0,  // 10.9 ft  2125 rpm 
     0, // 15 ft  2275 rpm 
     0, // 24 ft 2575 rpm 
     0, 
     0
   };

  private static final LinearRegression speedAtDistance = new LinearRegression(measuredDistancesMeters, measuredRPMs);

  private static final int VELOCITY_SLOT = 0;

  public Shooter(Vision vision) {

    SmartDashboard.putNumber("Shooter Velocity Setpoint", 0.0);
    SmartDashboard.putNumber("Shooter FF", 0.0);
    SmartDashboard.putNumber("Shooter P", 0.0);
    SmartDashboard.putNumber("Shooter Ramp Rate", 0.0);
    SmartDashboard.putNumber("Shooter speed", 0.0);

    this.vision = vision;

    controllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap.SHOOTER_MAIN,
        RobotMap.SHOOTER_FOLLOWER, true, Motor_Type.NEO);

    mainController = controllers.get(0);
    followerController = controllers.get(1);

    shooter_mainEncoder = mainController.getEncoder();
    shooter_mainPIDController = mainController.getPIDController();

    shooter_mainPIDController.setP(V_P, VELOCITY_SLOT);
    shooter_mainPIDController.setI(V_I, VELOCITY_SLOT);
    shooter_mainPIDController.setD(V_D, VELOCITY_SLOT);
    shooter_mainPIDController.setFF(V_FF, VELOCITY_SLOT);
  }

  public double getVelocity() {
    return shooter_mainEncoder.getVelocity();
  }

  public void run() {
    SmartDashboard.putNumber("Stage 2 speed", targetRPM + speedAdjust);
    shooter_mainPIDController.setReference(targetRPM + speedAdjust, ControlType.kVelocity);
  }

  /**
   * @param setRamp true if we want a ramp rate (use this for getting the shooter
   *                up to speed), false when we're ready to shoot and don't need
   *                one
   */
  public void setRampRate(boolean setRamp) {
    if (setRamp) {
      mainController.setClosedLoopRampRate(RAMP_RATE);
    } else {
      mainController.setClosedLoopRampRate(0);
    }
  }

  public void setTargetRPM(double targetRPM) {
    this.targetRPM = targetRPM;
  }

  public double getDefaultRPM(){
    return DEFAULT_SPEED;
  }

  /**
   * 
   * @param diff The amount to change the current RPM adjust by, positive for increasing and negative to decrease
   */
  public void adjustRPMAdjuster(double diff) {
    if(((diff > 0 && speedAdjust < 400) || (diff < 0 && speedAdjust > -400))){
      this.speedAdjust += diff;
      if(shooter_mainEncoder.getVelocity() > 300){
        run();
      }

    }
  }

  /**
   * 
   * @param distance In meters, the distance we are shooting at
   * @return The predicted "best fit" RPM for the motors to spin at based on the distance,
   * calculated from the linear regression
   */
   double getTargetRPMForDistance(double distance){
    double predictedVal = speedAtDistance.predict(distance);
    double expectedSpeed = Math.max(Math.min(predictedVal, MAX_RPM), MIN_RPM);
    SmartDashboard.putNumber("expectedSpeed", expectedSpeed);
    SmartDashboard.putNumber("predictedVal", predictedVal);
    SmartDashboard.putNumber("distance", distance);
    return expectedSpeed;
  }

  public void stop() {
    shooter_mainPIDController.setReference(0, ControlType.kDutyCycle);
    setTargetRPM(0);
  }

  public boolean isUpToSpeed() {
    return MathUtils.doublesEqual(getVelocity(), targetRPM + this.speedAdjust, 200); // TODO: margin of error
  }

  public void test() {
    shooter_mainPIDController.setReference(SmartDashboard.getNumber("Stage 2 Velocity Setpoint", 0.0), ControlType.kVelocity);
    SmartDashboard.putNumber("Stage 2 speed", mainController.getEncoder().getVelocity());
  }

  @Override
  public HealthState checkHealth() {
    if (isSparkMaxErrored(mainController) || isSparkMaxErrored(followerController)) {
      return HealthState.RED;
    }
    return HealthState.GREEN;
  }

  @Override
  public void mustangPeriodic() {
    // if (isShooting()){
    //   ballHasBeenShot = false;
    // } else if (!ballHasBeenShot && !isShooting()) {
    //   ballHasBeenShot = true;
    // }
    //if(targetRPM != 0){
      double distance = vision.getDistanceToTargetM();
      if (distance != RobotConstants.VISION_ERROR_CODE) {
        double targetRPM = getTargetRPMForDistance(distance);
        setTargetRPM(targetRPM);
        run();
      }
    //}   
    // SmartDashboard.putNumber("Shooter speed", mainController.getEncoder().getVelocity());
  }

  /*
  *Sets the RPM
  */
  public void setRPMForDistance(double distance) {
    Logger.consoleLog("Shooter distance to target %s", distance);
    double target = getTargetRPMForDistance(distance);
    Logger.consoleLog("Shooter RPM should be %s", target);
    setTargetRPM(target);
  }

  public boolean isShooting() {
    double current = mainController.getOutputCurrent();
    if (current > 0.2) {
      if (current >= NORMAL_CURRENT) {
        shootingCurrentCount++;
      } else {
        shootingCurrentCount = 0;
      }
      if (shootingCurrentCount >= 1) {
        return true;
      }
    }
    return false;
}
}
