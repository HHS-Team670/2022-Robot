
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
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.dataCollection.sensors.DIOUltrasonic;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.commands..OverrideDynamicRPM;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents a  with 2 NEOs.
 * Shoots using Vision, Ultrasonic, and manual overrides.
 * 
 * @author wilsonpotato, gentleTiger123, arghunter, smishra467
 */
public class  extends MustangSubsystemBase {

    private SparkMAXLite mainController, followerController;
    private List<SparkMAXLite> controllers;

    private RelativeEncoder _mainEncoder;
    private SparkMaxPIDController _mainPIDController;

    private double targetRPM = 0;
    private static double DEFAULT_SPEED = 2200;

    private static double MIN_RPM = 0;
    private static double MAX_RPM = 5600;

    private double waitTime = 2;

    private double speedAdjust = 0; // By default, we don't adjust, but this may get set later

    private int shootingCurrentCount = 0;

    private static final double NORMAL_CURRENT = 0;

    private static final double V_P = 0.000035;
    private static final double V_I = 0;
    private static final double V_D = 0.00004;
    private static final double V_FF = 0.000175;
    private static final double RAMP_RATE = 0.0;

    private double MIN_RUNNING_RPM = 0.0;
    private double MAX_RPM_ADJUSTMENT = 0.0;
    private double INITIAL_DIFF = 0;
    private static double SPEED_ALLOWED_ERROR = 100.0;
    private static double SHOOTING_CURRENT = 0.0;
    private static double manual_velocity;

    private boolean rpmVelSetCorrectly = true;

    private static DIOUltrasonic ultrasonic = new DIOUltrasonic(RobotMap._ULTRASONIC_TPIN,
            RobotMap._ULTRASONIC_EPIN);

    private static final double MIN_DISTANCE_HIGH_METERS = 1.60655;

    private static final int VELOCITY_SLOT = 0;
    private Vision vision;

    private boolean useDynamicSpeed = true;

    private MustangController mController;

    private ConveyorSystem conveyor;

    private boolean foundTarget;

    public (Vision vision, MustangController mController, ConveyorSystem conveyor) {
        this.vision = vision;
        setName("");
        setLogFileHeader(" Velocity Setpoint", " velocity", "Speed", "Ultrasonic distance", "P", "I", "D", "FF", "Ramp Rate");
        controllers = SparkMAXFactory.buildFactorySparkMAXPair(RobotMap._MAIN,
                RobotMap._FOLLOWER, true, Motor_Type.NEO);

        mainController = controllers.get(0);
        followerController = controllers.get(1);

        _mainEncoder = mainController.getEncoder();
        _mainPIDController = mainController.getPIDController();

        _mainPIDController.setP(V_P, VELOCITY_SLOT);
        _mainPIDController.setI(V_I, VELOCITY_SLOT);
        _mainPIDController.setD(V_D, VELOCITY_SLOT);
        _mainPIDController.setFF(V_FF, VELOCITY_SLOT);

        ultrasonic.setUltrasonicAutomaticMode(true);

        this.mController = mController;
        this.conveyor = conveyor;
        SmartDashboard.putString("overrided-rpm", "NOT OVERRIDED");
    
    }

    public double getVelocity() {
        return _mainEncoder.getVelocity();
    }

    public void run() {
        _mainPIDController.setReference(targetRPM + speedAdjust, ControlType.kVelocity);
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getDefaultRPM() {
        return DEFAULT_SPEED;
    }

    public boolean isRPMSetCorrectly(){
        return rpmVelSetCorrectly;
    }

    public void initDefaultCommand() {
        MustangScheduler.getInstance().setDefaultCommand(this, new OverrideDynamicRPM(this, mController));
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
            if (_mainEncoder.getVelocity() > MIN_RUNNING_RPM) {
                run();
            }
        }
    }

    /**
     * Gets target RPM for low goal based on the given distance using a prediction function
     * 
     * @param distance In meters, the distance we are shooting at
     * @return The predicted "best fit" RPM for the  to spin at based on the given distance.
     */
    public double getTargetRPMForLowGoalDistance(double distance) {
        double predictedVal = ((224 * distance) + 1517);
        double expectedSpeed = Math.max(Math.min(predictedVal, MAX_RPM), MIN_RPM);
        return expectedSpeed;
    }

    /**
     * Gets target RPM for high goal based on the given distance using regression
     * 
     * @param distance In meters, the distance we are shooting at
     * @return The predicted "best fit" RPM for the motors to spin at based on the
     *         distance, calculated from the linear regression.
     */
    public double getTargetRPMForHighGoalDistance(double distance) {
        double predictedVal = (300.674 * distance) + 2652.62; // These values worked best at SVR
        if(distance < 3){
            predictedVal += 250;
        }
        
        double expectedSpeed = Math.max(Math.min(predictedVal, MAX_RPM), MIN_RPM);
        return expectedSpeed;
    }

    public void stop() {
        _mainPIDController.setReference(0, ControlType.kDutyCycle);
    }

    public void idle() {
        _mainPIDController.setReference(3000, ControlType.kVelocity);
    }

    public boolean isUpToSpeed() {
        return Math.abs(getVelocity() - (targetRPM + this.speedAdjust)) < SPEED_ALLOWED_ERROR; // margin of error
    }

    public void test() {
        _mainPIDController.setReference(SmartDashboard.getNumber(" Velocity Setpoint", manual_velocity), ControlType.kVelocity);
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
        if (conveyor.getBallCount() > 0) {

            if (!vision.LEDSOverriden())
                vision.switchLEDS(true);

            if (vision.getDistanceToTargetM() != RobotConstants.VISION_ERROR_CODE)
                foundTarget = true;
            else
                foundTarget = false;

        } else {
            if (!vision.LEDSOverriden())
                vision.switchLEDS(true);
            foundTarget = false;
        }

        if(!isRPMSetCorrectly() && conveyor.getStatus().equals("Shooting")){
            conveyor.stopAll();
        }
    }

    public boolean foundTarget() {
        return foundTarget;
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
     * If vision works:
     *   Gets the distance to target from vision,
     *   predicts the RPM based off the distance,
     *   and sets that as the Target RPM.
     * If vision doesn't work:
     *   it tries to use the ultrasonic sensors
     * If that doesn't work either, then it will run the  at default speed
     */

    public double setRPM() {
        double targetRPM = 0;
        String SpeedChooser = "Manual (dynamicSpeed == false)"; //For debugging purposes. Uncomment the print to SmartDashboard at the bottom of this method.
        
        // If using vision or ultrasonic...
        if (useDynamicSpeed) {
            double distanceToTarget = RobotConstants.VISION_ERROR_CODE;

            if (foundTarget) {
                distanceToTarget = vision.getDistanceToTargetM();
                targetRPM = getTargetRPMForHighGoalDistance(distanceToTarget);
                SpeedChooser = "Vision";
            }
            if (Math.abs(distanceToTarget - RobotConstants.VISION_ERROR_CODE) < 10
                    || distanceToTarget < getMinHighDistanceInMeter()) { // double comparison
                distanceToTarget = getUltrasonicDistanceInMeters();
                targetRPM = getTargetRPMForLowGoalDistance(distanceToTarget);
                SpeedChooser = "Ultrasonic";
            }
            rpmVelSetCorrectly = true;
        } else { // If using a manual override...
            targetRPM = getTargetRPM();
            rpmVelSetCorrectly = true;
            SpeedChooser = "Manual (dynamicSpeed == false)";
        }
        
        // If target RPM somehow got set to an invalid number...
        if (targetRPM <= 0 || targetRPM > MAX_RPM) {
            targetRPM = 0;
            rpmVelSetCorrectly = false;
        }

        //SmartDashboard.putString("-speed-chooser", SpeedChooser);
        setTargetRPM(targetRPM);
        return targetRPM;
    }

    public void useDynamicSpeed(boolean isDynamic) {
        this.useDynamicSpeed = isDynamic;
    }

    public boolean isUsingDynamicSpeed() {
        return useDynamicSpeed;
    }

    public double getUltrasonicDistanceInMeters() {
        double dist = Units.inchesToMeters(ultrasonic.getDistance());
        if (dist <= 0.8)
            return dist;
        else
            return RobotConstants.VISION_ERROR_CODE;
    }

    public double getMinHighDistanceInMeter() {
        return MIN_DISTANCE_HIGH_METERS;
    }

    public void setLED(boolean on) {
        vision.switchLEDS(on);
    }

    public double getWaitTime() {
        return waitTime;
    }

    public void setWaitTime(double waitTime) {
        this.waitTime = waitTime;
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber(" Velocity Setpoint", manual_velocity);
        SmartDashboard.putNumber(" velocity", getVelocity());
        SmartDashboard.putNumber("_P", V_P);
        SmartDashboard.putNumber("_I", V_I);
        SmartDashboard.putNumber("_D", V_D);
        SmartDashboard.putNumber("_FF", V_FF);
        SmartDashboard.putNumber(" Ramp Rate", RAMP_RATE);
        SmartDashboard.putNumber(" speed", targetRPM);
        SmartDashboard.putNumber("Ultrasonic Distance", getUltrasonicDistanceInMeters());

        double VelocitySetpoint = manual_velocity;
        double Velocity = getVelocity();
        // writeToLogFile(VelocitySetpoint, Velocity, targetRPM, getUltrasonicDistanceInMeters(), V_P, V_I, V_D, V_FF, RAMP_RATE);
    }
}

/** Data for regression. Not directly used in code, but these should be saved, according to Laksh
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
    2.97,
    3.13055,
    3.66,
    4.54,
    4.65455
};

private static final double[] MEASURED_HIGH_RPM = {
    3150,
    3350,
    3650,
    3700,
    3850,
    3900,
    4200,
    4500
};

private static final double[] MEASURED_DISTANCE_HIGH_METER_RISKY = {
    1.60655,
    1.91135,
    2.21615,
    2.5,
    2.97,
    3.13055,
    3.66,
    3.99,
    4.54,
    4.65455, 
    4.9,
    5.2,
    5.5,
    5.8,
    6.1,
    6.4
};

private static final double[] MEASURED_HIGH_RPM_RISKY = {
    3150,
    3350,
    3650,
    3650,
    3700,
    3850,
    3900,
    4200,
    4200,
    4500,
    4700,
    4975,
    5200,
    5250,
    5315,
    5250,
};

private static final LinearRegression speedAtDistanceForHighGoal = new LinearRegression(
        MEASURED_DISTANCE_HIGH_METER,
        MEASURED_HIGH_RPM);
 */