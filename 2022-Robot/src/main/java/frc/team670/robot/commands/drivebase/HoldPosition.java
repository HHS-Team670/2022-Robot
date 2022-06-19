package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Calls the drivebase's holdPosition() method until the driver attempts to start driving again
 * 
 * @author lakshbhambhani
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

    @Override
    public void initialize() {
        driveBase.holdPosition();
    }

    @Override
    public void execute() {
        driveBase.getDriveTrain().feedWatchdog();
    }

    // Finished when controller has input
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
    }

}