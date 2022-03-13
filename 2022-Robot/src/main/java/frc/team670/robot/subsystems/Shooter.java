
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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.dataCollection.sensors.DIOUltrasonic;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.math.interpolable.LinearRegression;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
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
    private static double DEFAULT_SPEED = 2200;

    private static double MIN_RPM = 0;
    private static double MAX_RPM = 4750;

    private double speedAdjust = 0; // By default, we don't adjust, but this may get set later

    private int shootingCurrentCount = 0;

    private static final double NORMAL_CURRENT = 0;

    private static final double V_P = 0.0001;
    private static final double V_I = 0.0;
    private static final double V_D = 0.0;
    private static final double V_FF = 0.00017618;
    private static final double RAMP_RATE = 0.0;

    private double MIN_RUNNING_RPM = 0.0;
    private double MAX_RPM_ADJUSTMENT = 0.0;
    private double INITIAL_DIFF = 0;
    private static double SPEED_ALLOWED_ERROR = 100.0;
    private static double SHOOTING_CURRENT = 0.0;
    private static double VELOCITY_FOR_RAMP_RATE = 10.0;
    private static double manual_velocity;

    private static DIOUltrasonic ultrasonic = new DIOUltrasonic(RobotMap.SHOOTER_ULTRASONIC_TPIN,
            RobotMap.SHOOTER_ULTRASONIC_EPIN);

    private static final double[] MEASURED_DISTANCE_LOW_METER = {
            0.18415,
            0.48895,
            0.79375,
            1.09855,
            1.40335,
            2.01295,
            2.62255,
            3.23215,
            3.84175
    };

    private static final double[] MEASURED_LOW_RPM = {
            1550,
            1600,
            1800,
            2000,
            2250,
            2600,
            2800,
            3000,
            3500
    };

    private static final double[] MEASURED_DISTANCE_HIGH_METER = {
            1.60655,
            1.91135,
            2.21615, 
            3.13055,
            4.04495,
            4.65455
    };

    private static final double[] MEASURED_HIGH_RPM = {
            3150,
            3350,
            3650, 
            3850,
            4200,
            4500
    };

    private static final LinearRegression speedAtDistanceForHighGoal = new LinearRegression(
            MEASURED_DISTANCE_HIGH_METER,
            MEASURED_HIGH_RPM);

    private static final int VELOCITY_SLOT = 0;
    private Vision vision;

    private boolean useDynamicSpeed = true;
    
    public Shooter(Vision vision) {
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

        ultrasonic.setUltrasonicAutomaticMode(true);

        debugSubsystem();
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
    public double getTargetRPMForLowGoalDistance(double distance) {
        double predictedVal = ((224 * distance) + 1517); //speedAtDistanceForLowGoal.predict(distance); direct function was working better than the regressor
        double expectedSpeed = Math.max(Math.min(predictedVal, MAX_RPM), MIN_RPM);
        SmartDashboard.putNumber("expectedSpeedLow", expectedSpeed);
        SmartDashboard.putNumber("predictedValLow", predictedVal);
        SmartDashboard.putNumber("distanceLow", distance);
        return expectedSpeed;
    }

    /**
     * 
     * @param distance In meters, the distance we are shooting at
     * @return The predicted "best fit" RPM for the motors to spin at based on the
     *         distance,
     *         calculated from the linear regression.
     */
    public double getTargetRPMForHighGoalDistance(double distance) {
        double predictedVal = speedAtDistanceForHighGoal.predict(distance) + 50; // adding 100 to shoot into the outer upper part of the upper hub
        double expectedSpeed = Math.max(Math.min(predictedVal, MAX_RPM), MIN_RPM);
        SmartDashboard.putNumber("expectedSpeedHigh", expectedSpeed);
        SmartDashboard.putNumber("predictedValHigh", predictedVal);
        SmartDashboard.putNumber("distanceHigh", distance);
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
        SmartDashboard.putNumber("udist", getUltrasonicDistanceInMeters());
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

    /**
     * If vision works, it gets the distance to target from vision,
     * predicts the RPM based off the distance,
     * and sets that as the Target RPM
     * If vision doesn't work, it tries to use the ultrasonic sensors
     * If that doesn't work either, then it will run the shooter at default speed
     */

    public void setRPM() {
        double targetRPM = getDefaultRPM();
        if (useDynamicSpeed) {
            double distanceToTarget = RobotConstants.VISION_ERROR_CODE;
            if (vision.hasTarget()) {
                distanceToTarget = vision.getDistanceToTargetM();
                SmartDashboard.putNumber("speed-chooser", 0);
            }
            if(Math.abs(distanceToTarget-RobotConstants.VISION_ERROR_CODE) < 10){ // double comparison
                distanceToTarget = getUltrasonicDistanceInMeters();
                SmartDashboard.putNumber("speed-chooser", 1);
            }
            SmartDashboard.putNumber("distance to target", distanceToTarget);
            if(Math.abs(distanceToTarget-RobotConstants.VISION_ERROR_CODE) < 10){  //double comparison
                setTargetRPM(getDefaultRPM());
                SmartDashboard.putNumber("speed-chooser", 2);
                return;
            }
            if (distanceToTarget < getMinHighDistanceInMeter()) {
                targetRPM = getTargetRPMForLowGoalDistance(distanceToTarget);
            } else {
                targetRPM = getTargetRPMForHighGoalDistance(distanceToTarget);
            }
        }
        if(targetRPM < 0 || targetRPM > MAX_RPM){
            targetRPM = getDefaultRPM();
        }
        setTargetRPM(targetRPM);

    }

    public void useDynamicSpeed(boolean isDynamic){
        this.useDynamicSpeed = isDynamic;
    }

    public boolean isUsingDynamicSpeed(){
        return useDynamicSpeed;
    }

    public double getUltrasonicDistanceInMeters(){
        double dist = Units.inchesToMeters(ultrasonic.getDistance());
        if(dist <= 0.8){
            return dist;
        }
        else{
            return RobotConstants.VISION_ERROR_CODE;
        }
    }

    public double getMinHighDistanceInMeter(){
        return MEASURED_DISTANCE_HIGH_METER[0];
    }

    public void setLED(boolean on) {
        vision.switchLEDS(on);
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Shooter Velocity Setpoint", manual_velocity);
        SmartDashboard.putNumber("Shooter FF", V_FF);
        SmartDashboard.putNumber("Shooter P", V_P);
        SmartDashboard.putNumber("Shooter Ramp Rate", RAMP_RATE);
        SmartDashboard.putNumber("Shooter speed", targetRPM);
        SmartDashboard.putNumber("Shooter velocity", getVelocity());
    }
}
