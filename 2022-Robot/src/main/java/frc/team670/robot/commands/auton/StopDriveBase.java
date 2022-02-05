
package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class StopDriveBase extends CommandBase implements MustangCommand{
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    
    private DriveBase driveBase;
    public StopDriveBase(DriveBase driveBase) {
        this.driveBase = driveBase;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
    }

    public void initialize() {
        super.initialize();
        driveBase.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

}