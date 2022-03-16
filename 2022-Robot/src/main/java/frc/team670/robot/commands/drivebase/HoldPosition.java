package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
public class HoldPosition extends CommandBase implements MustangCommand {

    private DriveBase driveBase;

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public HoldPosition(DriveBase driveBase) {
        this.driveBase = driveBase;
        addRequirements(driveBase);
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
        driveBase.holdPosition();
    }

    @Override
    public void execute() {
        driveBase.getDriveTrain().feedWatchdog();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(driveBase.getMustangController().getRightStickX()) > 0.1
                || Math.abs(driveBase.getMustangController().getLeftStickX()) > 0.1
                || Math.abs(driveBase.getMustangController().getLeftStickY()) > 0.1) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.releasePosition();
        // driveBase.initDefaultCommand();
    }

}