
package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Calls drivebase.stop() immediately
 */
public class StopDriveBase extends CommandBase implements MustangCommand{
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    
    private DriveBase driveBase;
    public StopDriveBase(DriveBase driveBase) {

        this.driveBase = driveBase;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);

        Logger.consoleLog("CALLED STOP DRIVE BASE");
        Logger.consoleLog("CALLED STOP DRIVE BASE");
    }

    public void initialize() {
        SmartDashboard.putString("Stop drive base called", "yea");

        super.initialize();
        driveBase.stop();
    }

    public boolean isFinished() {
        return true;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

}