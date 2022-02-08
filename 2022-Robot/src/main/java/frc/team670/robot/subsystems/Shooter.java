
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.math.interpolable.LinearRegression;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents a shooter with 2 NEOs
 * 
 * @author wilsonpotato, gentleTiger123, arghunter, smishra467
 */
public class Shooter extends MustangSubsystemBase {

    private SparkMAXLite mainController, followerController;
    private List<SparkMAXLite> controllers;

    private RelativeEncoder shooter_mainEncoder;
    private SparkMaxPIDController shooter_mainPIDController;

    private double targetRPM = 0;
    private static double DEFAULT_SPEED = 2500;

    private static double MIN_RPM = 0;
    private static double MAX_RPM = 0;

    private double speedAdjust = 0; // By default, we don't adjust, but this may get set later

    private static double MAX_SHOT_DISTANCE_METERS = 0;

    private static final double PULLEY_RATIO = 0;

    private boolean ballHasBeenShot;
    private int shootingCurrentCount = 0;

    private static final double NORMAL_CURRENT = 0;

    private static final double V_P = 0.00009;
    private static final double V_I = 0.0;
    private static final double V_D = 0.0;
    private static final double V_FF = 0.00017618;
    private static final double RAMP_RATE = 0.0;

    private double MIN_RUNNING_RPM = 0.0;
    private double MAX_RPM_ADJUSTMENT = 0.0;
    private double INITIAL_DIFF = 0;

    private static double SPEED_ALLOWED_ERROR = 10.0;

    private static double SHOOTING_CURRENT = 0.0;

    private static double VELOCITY_ALLOWED_ERROR = 10.0;

    private static double VELOCITY_FOR_RAMP_RATE = 10.0;

    private static double manual_velocity;

    //private Vision vision;

    private static final double[] measuredDistancesMeters = {
            0,
            0,
            0,
            0,
            0,
    };

    private static final double[] measuredRPMs = {
            0,
            0,
            0,
            0,
            0
    };

    private static final LinearRegression speedAtDistance = new LinearRegression(measuredDistancesMeters, measuredRPMs);

    private static final int VELOCITY_SLOT = 0;

    public Shooter() {
        SmartDashboard.putNumber("Shooter Velocity Setpoint", manual_velocity);
        SmartDashboard.putNumber("Shooter FF", V_FF);
        SmartDashboard.putNumber("Shooter P", V_P);
        SmartDashboard.putNumber("Shooter Ramp Rate", RAMP_RATE);
        SmartDashboard.putNumber("Shooter speed", targetRPM);

        //this.vision = vision;

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
        SmartDashboard.putNumber("Shooter target speed", targetRPM + speedAdjust);
        if (getVelocity() < VELOCITY_FOR_RAMP_RATE) {
            setRampRate(true);
        } else {
            setRampRate(false);
        }

        shooter_mainPIDController.setReference(targetRPM + speedAdjust, ControlType.kVelocity);
    }

    /**
     * @param setRamp true if we want a ramp rate (use this for getting the shooter
     *                up to speed), false when we're ready to shoot and don't need
     *                one
     */
    private void setRampRate(boolean setRamp) {
        if (setRamp) {
            mainController.setClosedLoopRampRate(RAMP_RATE);
        } else {
            mainController.setClosedLoopRampRate(0);
        }
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public double getDefaultRPM() {
        return DEFAULT_SPEED;
    }

    /**
     * 
     * @param diff The amount to change the current RPM adjust by, positive for
     *             increasing and negative to decrease
     */
    public void adjustRPMAdjuster(double diff) {
        if (((diff > INITIAL_DIFF && speedAdjust < MAX_RPM_ADJUSTMENT)
                || (diff < INITIAL_DIFF && speedAdjust > -(MAX_RPM_ADJUSTMENT)))) {
            this.speedAdjust += diff;
            if (shooter_mainEncoder.getVelocity() > MIN_RUNNING_RPM) {
                run();
            }
        }
    }

    /**
     * 
     * @param distance In meters, the distance we are shooting at
     * @return The predicted "best fit" RPM for the motors to spin at based on the
     *         distance,
     *         calculated from the linear regression.
     */
    double getTargetRPMForLowGoalDistance(double distance) {
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
        return Math.abs(getVelocity() - (targetRPM + this.speedAdjust)) < SPEED_ALLOWED_ERROR; // margin of
                                                                                                         // error
    }

    public void test() {
        shooter_mainPIDController.setReference(SmartDashboard.getNumber("Shooter Velocity Setpoint", manual_velocity),
                ControlType.kVelocity);
        SmartDashboard.putNumber("Shooter speed", mainController.getEncoder().getVelocity());
    }

    @Override
    public HealthState checkHealth() {
        if (mainController.isErrored() || followerController.isErrored()) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // double distance = vision.getDistanceToTargetM();
        // if (distance != RobotConstants.VISION_ERROR_CODE) {
        //     double targetRPM = getTargetRPMForLowGoalDistance(distance);
        //     setTargetRPM(targetRPM);
        //     run();
        // }
        SmartDashboard.putNumber("Shooter velocity", getVelocity());

        if (Math.abs(getVelocity() - targetRPM) < VELOCITY_ALLOWED_ERROR) {
            setRampRate(false);
        }

    }

    /**
     * @param distance In meters, the distance we are shooting at
     *                 Predicts the target RPM based off the distance
     *                 and sets it as the target RPM
     */
    public void setRPMForDistance(double distance) {
        double RPMtarget = getTargetRPMForLowGoalDistance(distance);
        setTargetRPM(RPMtarget);
    }

    public boolean isShooting() {
        double current = mainController.getOutputCurrent();
        if (current > SHOOTING_CURRENT) {
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
