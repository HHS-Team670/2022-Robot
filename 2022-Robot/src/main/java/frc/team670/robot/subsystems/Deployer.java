package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the deployer for the intake
 * 
 * @author lakshbhambhani
 */
public class Deployer extends SparkMaxRotatingSubsystem {

    private DutyCycleEncoder absEncoder;

    private static final double ABSOLUTE_ENCODER_POSITION_AT_FLIPOUT_ZERO = -0.086; // From 2/17
    private static final double ABSOLUTE_ENCODER_GEAR_RATIO = 25.76582278;

    private static final double MAX_FLIPOUT_ROTATIONS = -8.142;
    public static final int FLIPOUT_GEAR_RATIO = 50;
    public static final int MAX_ACCEL_DOWNWARDS = 700;
    public static final int MAX_ACCEL_UPWARDS = 1900;

    private boolean isDeployed = false;

    /**
     * PID and SmartMotion constants for the flipout rotator go here.
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.FLIP_OUT;
        }

        public int getSlot() {
            return 0;
        }

        public MotorConfig.Motor_Type getMotorType() {
            return MotorConfig.Motor_Type.NEO;
        }

        public double getP() {
            return 0.00015; // Good enough for 2/17
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0;
        }

        public double getFF() { // Good enough for 2/17
            return 0.000176;
        }

        public double getIz() {
            return 0;
        }

        public double getMaxOutput() {
            return 1;
        }

        public double getMinOutput() {
            return -1;
        }

        public double getMaxAcceleration() {
            return 1900;
        }

        public double getAllowedError() {
            return 0.25;
        }

        public boolean enableSoftLimits() {
            return false;
        }

        public float[] setSoftLimits() {
            return null;
        }

        public int getContinuousCurrent() {
            return 20;
        }

        public int getPeakCurrent() {
            return 80;
        }

        public double getRotatorGearRatio() {
            return FLIPOUT_GEAR_RATIO;
        }

        public IdleMode setRotatorIdleMode() {
            return IdleMode.kCoast;
        }

        @Override
        public double getMaxRotatorRPM() {
            return 3500;
        }

        @Override
        public double getMinRotatorRPM() {
            return 0;
        }

    }

    public static final Config FLIPOUT_CONFIG = new Config();

    public Deployer() {
        super(FLIPOUT_CONFIG);
        absEncoder = new DutyCycleEncoder(RobotMap.FLIP_OUT_ABS_ENCODER);
        setEncoderPositionFromAbsolute();
        enableCoastMode();
    }

    public double getSpeed() {
        return rotator.get();
    }

    public double getAbsoluteEncoderRotations() {
        return absEncoder.get();
    }

    /**
     * Sets the rotator encoder's reference position to the constant obtained from
     * the absolute encoder corresponding to that position.
     */
    public void setEncoderPositionFromAbsolute() {
        clearSetpoint();
        rotator_encoder.setPosition(
                -1 * (getAbsoluteEncoderRotations() - ABSOLUTE_ENCODER_POSITION_AT_FLIPOUT_ZERO) * ABSOLUTE_ENCODER_GEAR_RATIO);
        Logger.consoleLog("Encoder position set: %s", rotator_encoder.getPosition());
    }

    /**
     * @return the position, in number of rotations of the flipout
     */
    public double getPosition() {
        return rotator_encoder.getPosition();
    }

    @Override
    public HealthState checkHealth() {
       
        if (rotator.isErrored()) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    /**
     * Converts an intake angle into rotations
     */
    private static int convertFlipoutDegreesToRotations(double degrees) {
        // If straight up is 0 and going forward is positive
        // percentage * half rotation
        return (int) ((degrees / 90) * MAX_FLIPOUT_ROTATIONS);
    }

    /**
     * Converts intake rotations into an angle
     */
    private static double convertFlipoutRotationsToDegrees(double rotations) {
        // If straight up is 0 and going forward is positive
        return ((90 * rotations) / MAX_FLIPOUT_ROTATIONS);
    }

    /**
     * Turns to a target angle the most efficient way.
     */
    @Override
    public void setSystemTargetAngleInDegrees(double angleDegrees) {
        angleDegrees = MathUtil.clamp(angleDegrees, 0, 90);
        double currentAngle = getCurrentAngleInDegrees();
        if(angleDegrees > currentAngle){
            rotator_controller.setSmartMotionMaxAccel(MAX_ACCEL_DOWNWARDS, this.SMARTMOTION_SLOT);
        }
        else{
            rotator_controller.setSmartMotionMaxAccel(MAX_ACCEL_UPWARDS, this.SMARTMOTION_SLOT);
        }
        setSystemMotionTarget(convertFlipoutDegreesToRotations(angleDegrees));
    }

    /**
     * @return the angle the flipout is currently turned to, between 0 and 90
     */
    @Override
    public double getCurrentAngleInDegrees() {
        return convertFlipoutRotationsToDegrees(getUnadjustedPosition());
    }

    @Override
    public void mustangPeriodic() {
        debugSubsystem();
    }

    public boolean hasReachedTargetPosition() {
        boolean hasReachedTarget = (MathUtils.doublesEqual(rotator_encoder.getPosition(), setpoint, ALLOWED_ERR));
        return hasReachedTarget;
    }

    public boolean deploy(boolean deploy){
        double angle = 0;
        if(deploy){
            angle = 90;
            isDeployed = true;
        }
        else{
            isDeployed = false;
        }
        setSystemTargetAngleInDegrees(angle);
        return hasReachedTargetPosition();
    }

    /**
     * @param true to set flipout to coast mode, false to set it to brake mode
     */
    public void setRotatorMode(boolean coast) {
        if (coast) {
            rotator.setIdleMode(IdleMode.kCoast);
        } else {
            rotator.setIdleMode(IdleMode.kBrake);
        }
    }

    public boolean isDeployed(){
        return (isDeployed && hasReachedTargetPosition());
    } 

    @Override
    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void moveByPercentOutput(double output) {
        // TODO Auto-generated method stub

    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("abs-Encoder", absEncoder.get());
        SmartDashboard.putNumber("rel-Encoder", this.rotator_encoder.getPosition());
        SmartDashboard.putNumber("rel-Encoder-vel", this.rotator_encoder.getVelocity());
        SmartDashboard.putNumber("angle", getCurrentAngleInDegrees());
    }
}