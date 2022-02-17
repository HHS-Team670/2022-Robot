package frc.team670.robot.commands.auton;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Starts flush with lower hub.
 * Scores 1 lower hub, then picks up one more.
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class ATarmacFlushed1Ball extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;

    public ATarmacFlushed1Ball(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter) {
        trajectory = PathPlanner.loadPath("ATarmacFlushed1Ball", 1.0, 0.5);

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
            new ParallelCommandGroup(
                getTrajectoryFollowerCommand(trajectory, driveBase)


            ),
            //shoot
            new AutoShootToIntake(conveyor, shooter, intake),
            getTrajectoryFollowerCommand(trajectory, driveBase),
            //intake
            new StopDriveBase(driveBase)
        );

    }

    @Override
    public void initialize() {
        super.initialize();
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}
