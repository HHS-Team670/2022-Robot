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
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.routines.shoot.WaitToShoot;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/*
 * BTarmac4BallTerminal   
    * Starts flush with the edge of B tarmac.
    * Picks up 1 additional ball and shoots both low.
    * Picks up 1 from the ground and 1 from terminal and shoots both low.
 * BTarmac4BallTerminal2Ball
    * Starts on the edge of B tarmac facing the middle ball.
    * Intakes middle ball, shoots high.
    * Goes to terminal and picks up 2 balls 
 * ATarmacEdge4Ball
    * Starts on the edge of the A tarmac with 1 ball. Scores a total of 4 balls.
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class FourBallPath extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory, trajectory2;
    private Pose2d targetPose, targetPose2;

    public FourBallPath(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter, Vision vision,
            String pathName) {
        
        // TODO: Check if using pathName + "P1" works, rather than
        //       using the if/else statements (this was changed while
        //       debugging the bot going in a circle on 2/19)

        // trajectory = PathPlanner.loadPath(pathName + "P1", 2.0, 1);
        // trajectory2 = PathPlanner.loadPath(pathName + "P2", 2.0, 1);

        if (pathName.equals("BTarmac4BallTerminal")) {
            trajectory = PathPlanner.loadPath("BTarmac4BallTerminalP1", 2.0, 1);
            trajectory2 = PathPlanner.loadPath("BTarmac4BallTerminalP2", 2.0, 1);
        }

        if (pathName.equals("BTarmacHighHubTerminal")) {
            trajectory = PathPlanner.loadPath("BTarmacHighHubTerminalP1", 2.0, 1);
            trajectory2 = PathPlanner.loadPath("BTarmacHighHubTerminalP2", 2.0, 1);
        }

        if (pathName.equals("ATarmacEdge4Ball")) {
            trajectory = PathPlanner.loadPath("ATarmacEdge4BallP1", 2.0, 1);
            trajectory2 = PathPlanner.loadPath("ATarmacEdge4BallP2", 2.0, 1);
        }

        double errorInMeters = 0.2;
        targetPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
        targetPose2 = trajectory2.getStates().get(trajectory2.getStates().size() - 1).poseMeters;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(vision, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
                new RunIntakeWithConveyor(intake, conveyor),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                getTrajectoryFollowerCommand(trajectory, driveBase),
                                getTrajectoryFollowerCommand(trajectory2, driveBase)),
                        new SequentialCommandGroup(
                                new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, -1.2, "upper"),
                                new AutoShootToIntake(driveBase, conveyor, shooter, intake, vision),
                                new WaitToShoot(driveBase, shooter, targetPose2, errorInMeters, 2,"lower"),
                                new ShootAllBalls(driveBase, conveyor, shooter, vision))), //TODO: test if ShootAllBalls works (rather than autoShootToIntake)
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