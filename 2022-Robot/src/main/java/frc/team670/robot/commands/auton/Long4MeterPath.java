package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Goes 4 meters forwards.
 * Useful for debugging.
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class Long4MeterPath extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;
    private DriveBase driveBase;

    public Long4MeterPath(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter) {
        // trajectory = PathPlanner.loadPath("Long4MeterPath", 1, 0.5);
        trajectory = PathPlanner.loadPath("SecondFourMeterPath", 2, 1);
        this.driveBase = driveBase;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);

        addCommands(
            getTrajectoryFollowerCommand(trajectory, driveBase),
            new StopDriveBase(driveBase)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
