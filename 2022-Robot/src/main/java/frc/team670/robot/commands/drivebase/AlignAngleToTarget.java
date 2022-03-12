package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;

/**
 * Rotates the turret to an angle with the goal of having the turret pointed in
 * the direction of the target for as much as possible.
 * 
 * @author ctychen
 */
public class AlignAngleToTarget extends CommandBase implements MustangCommand {

    private DriveBase driveBase;
    private Vision vision;
    private double targetAngle;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private double relativeYawToTarget;

    double prevCapTime;

    private final double ANGULAR_P = 0.03;
    private final double ANGULAR_D = 0.0;
    private PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    private double rotationSpeed;
    private double heading;

    public AlignAngleToTarget(DriveBase driveBase, Vision vision) {
        this.driveBase = driveBase;
        this.vision = vision;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    /**
     * Gets the relative angle to the target from camera when possible, converts to
     * absolute angle, and sets that to the turret's target position.
     * 
     * If vision data is unavailable, use position on the field to determine the
     * approximate angle to the target and account for heading to turn.
     * 
     */
    @Override
    public void initialize() {
        
        prevCapTime = 0.0;
        relativeYawToTarget = vision.getAngleToTarget();
        double capTime = vision.getVisionCaptureTime();
        heading = driveBase.getHeading();
        if (capTime != prevCapTime && relativeYawToTarget != RobotConstants.VISION_ERROR_CODE) {
            targetAngle = heading - relativeYawToTarget;
            prevCapTime = capTime;
        }
        turnController = new PIDController(SmartDashboard.getNumber("p align", 0), 0, SmartDashboard.getNumber("d align", 0));
        turnController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        heading = driveBase.getHeading();
        rotationSpeed = MathUtil.clamp(turnController.calculate(heading, targetAngle), -0.3, 0.3); // Max speed can be
                                                                                                   // 0.3
        if(rotationSpeed > 0 && rotationSpeed < 0.15){
            rotationSpeed = 0.15;
        } 
        else if(rotationSpeed < 0 && rotationSpeed > -0.15){
            rotationSpeed = -0.15;
        }  

        Logger.consoleLog("Rotation speed: %s target angle: %s, heading %s", rotationSpeed, targetAngle, heading);
        driveBase.curvatureDrive(0, heading < targetAngle ? -rotationSpeed : -rotationSpeed, true); // 0.3 is just a
                                                                                                    // constant safe
        // quick-turn rotational speed
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(driveBase.getHeading() - targetAngle) <= 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.stop();
        // driveBase.initDefaultCommand();
    }

}