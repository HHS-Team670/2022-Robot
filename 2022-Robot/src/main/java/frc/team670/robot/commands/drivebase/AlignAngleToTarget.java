package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.PIDConstantSet;
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

    protected DriveBase driveBase;
    protected Vision vision;
    protected double targetAngle;
    protected static final double MIN_RPM = 0.152;
    private Timer timer = new Timer();
    protected Map<MustangSubsystemBase, HealthState> healthReqs;

    protected double relativeYawToTarget;

    double prevCapTime;
    private int ANGLE_ALIGN_CANCEL=4;
    PIDConstantSet ANGULAR = new PIDConstantSet(0.014, 0.000001, 0.000003);
    protected PIDController turnController = new PIDController(ANGULAR.P, ANGULAR.I, ANGULAR.D);

    protected double rotationSpeed;
    protected double heading;

    protected boolean foundTarget = false;

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
            turnController = new PIDController(ANGULAR.P, ANGULAR.I, ANGULAR.D);
            turnController.enableContinuousInput(-180, 180);
            foundTarget = true;
        }
        timer.start();
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

        if (rotationSpeed > 0 && rotationSpeed < MIN_RPM) // Minimum rotation speed (magnitude)
            rotationSpeed = MIN_RPM;
        else if (rotationSpeed < 0 && rotationSpeed > -MIN_RPM)
            rotationSpeed = -MIN_RPM;
        

        driveBase.curvatureDrive(0, -rotationSpeed, true);
    }

    @Override
    public boolean isFinished() {
        if(timer.advanceIfElapsed(ANGLE_ALIGN_CANCEL)){
            timer.reset();
            timer.stop();
            return true;
        }
        return (!foundTarget || ((Math.abs(driveBase.getHeading() - targetAngle) <= 1) && driveBase.getWheelSpeeds().rightMetersPerSecond < 0.1));
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.stop();
        SmartDashboard.putBoolean("aligned", true);
    }

}