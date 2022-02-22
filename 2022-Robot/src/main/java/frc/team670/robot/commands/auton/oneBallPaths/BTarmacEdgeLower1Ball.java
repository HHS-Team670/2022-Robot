package frc.team670.robot.commands.auton.oneBallPaths;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.auton.StopDriveBase;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;


/**
 * Starts at the edge of the B tarmac, shoots 1 upper, and picks up 1 more
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class BTarmacEdgeLower1Ball extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;

    public BTarmacEdgeLower1Ball(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter, Vision vision) {
        trajectory = PathPlanner.loadPath("BTarmacEdgeLower1Ball", 2.0, 1);
        
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(vision, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
            new AutoShootToIntake(driveBase, conveyor, shooter, intake, vision),
            getTrajectoryFollowerCommand(trajectory, driveBase),
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