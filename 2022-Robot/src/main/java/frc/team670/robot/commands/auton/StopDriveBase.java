
package frc.team670.robot.commands.auton;

import java.util.Map;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class StopDriveBase extends InstantCommand {

    private DriveBase driveBase;
    public StopDriveBase(DriveBase driveBase) {
        this.driveBase = driveBase;
    }

    public void initialize() {
        super.initialize();
        driveBase.stop();
    }

}