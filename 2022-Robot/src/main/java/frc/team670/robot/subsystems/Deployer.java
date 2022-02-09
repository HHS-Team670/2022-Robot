package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.GravitySparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;


public class Deployer extends GravitySparkMaxRotatingSubsystem {
    private static final int FORWARD_SOFT_LIMIT = 0;
    private static final int REVERSE_SOFT_LIMIT = 0;
    private static final int FORWARD_HARD_LIMIT = 0;
    private static final int REVERSE_HARD_LIMIT = 0;
    private static final boolean REVERSED = false;
    private static final int STOWED_SETPOINT = 0;
    private static final int DEPLOYED_SETPOINT = 4;
    private static final int DEPLOYED_DISTANCE_FROM_INTAKE = 0; // Determine experimentally like all the other constants
    
    private double intakeAngleDegreesFromVertical;

    public Deployer(SparkMAXLite motor, int PIDSlot){
        super(motor, RobotConstants.ARBITRARY_FF_DEPLOYER, FORWARD_SOFT_LIMIT, REVERSE_SOFT_LIMIT, FORWARD_HARD_LIMIT, REVERSE_HARD_LIMIT, REVERSED);
        setIntakeAngleDegreesFromVertical();
    }

    public void deployIntake() {
        super.setpoint = DEPLOYED_SETPOINT;
    }
    
    public void retractIntake() {
        super.setpoint = STOWED_SETPOINT;
    }

    public void setIntakeAngleDegreesFromVertical() {
        intakeAngleDegreesFromVertical = (((double) super.offsetFromEncoderZero) / DEPLOYED_DISTANCE_FROM_INTAKE) * 90.0;
    }

    public boolean isDeployed() {
        setIntakeAngleDegreesFromVertical();
        if (intakeAngleDegreesFromVertical > 80 && intakeAngleDegreesFromVertical < 95) {
            return true;
        }
        return false;
    }

    public boolean isAtTarget() {
        if (super.setpoint == super.offsetFromEncoderZero) {
            return true;
        }
        return false;
    }

    @Override
    protected double getArbitraryFeedForwardAngleMultiplier() {
        return Math.sin(intakeAngleDegreesFromVertical);
    }
}
