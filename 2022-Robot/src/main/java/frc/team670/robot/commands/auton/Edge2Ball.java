package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.routines.shoot.WaitToShoot;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Works for any of the 3 edge2ball paths
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class Edge2Ball extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;
    private Pose2d targetPose;

    // path names: "ATarmacEdge2Ball", "BTarmacEdgeCenter2Ball", "BTarmacEdgeLower2Ball"
    public Edge2Ball(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter, String pathName) {
        trajectory = PathPlanner.loadPath(pathName, 1.0, 0.5);
        double errorInMeters = 0.5;
        targetPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
                new RunIntakeWithConveyor(intake, conveyor),
                getTrajectoryFollowerCommand(trajectory, driveBase),
                new SequentialCommandGroup(
                        new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, "upper"),
                        new ShootAllBalls(conveyor, shooter)),
                new StopDriveBase(driveBase));
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
