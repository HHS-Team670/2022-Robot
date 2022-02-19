package frc.team670.robot.commands.auton;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.conveyor.StopConveyor;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.routines.shoot.WaitToShoot;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Starts on the edge of the A tarmac with 1 ball. Scores a total of 4 balls.
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class ATarmacEdge4Ball extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory1, trajectory2;
    Pose2d targetPose1, targetPose2;

    public ATarmacEdge4Ball(DriveBase driveBase, Intake intake, Shooter shooter, ConveyorSystem conveyor) {
        
        trajectory1 = PathPlanner.loadPath("ATarmacEdge4BallP1", 2.0, 1);
        trajectory2 = PathPlanner.loadPath("ATarmacEdge4BallP2", 2.0, 1);
        double errorInMeters = 0.5;
        targetPose1 = trajectory1.getStates().get(trajectory1.getStates().size() - 1).poseMeters;
        targetPose2 = trajectory2.getStates().get(trajectory2.getStates().size() - 1).poseMeters;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);

        driveBase.resetOdometry(trajectory1.getStates().get(0).poseMeters);
        addCommands(
            new RunIntakeWithConveyor(intake, conveyor),

            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    getTrajectoryFollowerCommand(trajectory1, driveBase),
                    getTrajectoryFollowerCommand(trajectory2, driveBase)
                ),
                
                new SequentialCommandGroup(
                    new WaitToShoot(driveBase, shooter, targetPose1, errorInMeters),
                    new AutoShootToIntake(conveyor, shooter, intake),
                    new WaitToShoot(driveBase, shooter, targetPose2, errorInMeters),
                    new ShootAllBalls(conveyor, shooter)
                )
            ),

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
