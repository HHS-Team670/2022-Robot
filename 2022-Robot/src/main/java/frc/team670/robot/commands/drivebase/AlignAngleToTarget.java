package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;

/**
 * Rotates the drivebase to the vision target.
 * If no vision target is detected during initialize(), the command exits immediately.
 * If vision is obstructed while execute() is being run, the drivebase will turn until
 *      it has reached the target angle calculated the last time vision saw the target.
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
    private double ANGULAR_I = 0.000001;
    private double ANGULAR_D = 0.000003;
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
        if(vision.hasTarget()){
            relativeYawToTarget = vision.getAngleToTarget();
            heading = driveBase.getHeading();
            targetAngle = heading - relativeYawToTarget;
            turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
            turnController.enableContinuousInput(-180, 180);
            foundTarget = true;
        }
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            relativeYawToTarget = vision.getAngleToTarget();
            targetAngle = heading - relativeYawToTarget;
            foundTarget = true;
        }
        // no need to set foundTarget false, or else the robot will stop turning if vision is obstructed temporarily

        heading = driveBase.getHeading();
        rotationSpeed = MathUtil.clamp(turnController.calculate(heading, targetAngle), -0.3, 0.3); // Speed is capped at > 0.3

        if (rotationSpeed > 0 && rotationSpeed < 0.15) // Minimum rotation speed (magnitude) of 0.15
            rotationSpeed = 0.15;
        else if (rotationSpeed < 0 && rotationSpeed > -0.15)
            rotationSpeed = -0.15;
        

        driveBase.curvatureDrive(0, heading < targetAngle ? -rotationSpeed : -rotationSpeed, true);
    }

    @Override
    public boolean isFinished() {
        return (!foundTarget || ((Math.abs(driveBase.getHeading() - targetAngle) <= 1) && driveBase.getWheelSpeeds().rightMetersPerSecond < 0.1));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.stop();
    }

}