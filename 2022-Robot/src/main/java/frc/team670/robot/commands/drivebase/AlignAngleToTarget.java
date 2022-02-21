package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
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

    private boolean validData = false;
    private double relativeYawToTarget;

    public AlignAngleToTarget(DriveBase driveBase, Vision vision) {
        addRequirements(driveBase);
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
        validData = true;
        if (relativeYawToTarget == RobotConstants.VISION_ERROR_CODE) {
            validData = false;
            return;
        }
        driveBase.cancelDefaultCommand();
    }

    @Override
    public void execute(){
        relativeYawToTarget = vision.getAngleToTarget();
        driveBase.curvatureDrive(0, relativeYawToTarget < 0 ? -0.3 : 0.3, true); // 0.3 is just a constant safe quick-turn rotational speed
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(relativeYawToTarget) <= 5 || !validData);
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.initDefaultCommand();
    }

}