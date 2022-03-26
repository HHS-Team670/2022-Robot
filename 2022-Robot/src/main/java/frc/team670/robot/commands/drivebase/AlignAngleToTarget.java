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
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;

/**
 * Rotates the drivebase to an angle
 * 
 * @author katia
 */
public class AlignAngleToTarget extends CommandBase implements MustangCommand {

    private DriveBase driveBase;
    private Vision vision;
    private double targetAngle;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private double relativeYawToTarget;

    double prevCapTime;

    private double ANGULAR_P = 0.014;
    private double ANGULAR_D = 0.0015;
    private PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    private double rotationSpeed;
    private double heading;

    private boolean foundTarget = false;

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
        ANGULAR_P = SmartDashboard.getNumber("Align P", ANGULAR_P);
        ANGULAR_D = SmartDashboard.getNumber("Align D", ANGULAR_D);
        if(vision.hasTarget()){
            relativeYawToTarget = vision.getLastValidAngleCaptured();
            heading = driveBase.getHeading();
            targetAngle = heading - relativeYawToTarget;
            turnController = new PIDController(ANGULAR_P, SmartDashboard.getNumber("Align I", 0), ANGULAR_D);
            turnController.enableContinuousInput(-180, 180);
            foundTarget = true;
        }
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            relativeYawToTarget = vision.getLastValidAngleCaptured();
            targetAngle = heading - relativeYawToTarget;
            foundTarget = true;
        }
        heading = driveBase.getHeading();
        rotationSpeed = MathUtil.clamp(turnController.calculate(heading, targetAngle), -0.3, 0.3); // Max speed can be
                                                                                                   // 0.3
        if (rotationSpeed > 0 && rotationSpeed < 0.15) {
            rotationSpeed = 0.15;
        } else if (rotationSpeed < 0 && rotationSpeed > -0.15) {
            rotationSpeed = -0.15;
        }

        Logger.consoleLog("Rotation speed: %s target angle: %s, heading %s", rotationSpeed, targetAngle, heading);
        driveBase.curvatureDrive(0, heading < targetAngle ? -rotationSpeed : -rotationSpeed, true); // 0.3 is just a
                                                                                                    // constant safe
        // quick-turn rotational speed
    }

    @Override
    public boolean isFinished() {
        return (((!foundTarget || ((Math.abs(driveBase.getHeading() - targetAngle) <= 1)
                && driveBase.getWheelSpeeds().rightMetersPerSecond < 0.1))));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.stop();
        // driveBase.initDefaultCommand();
    }

}