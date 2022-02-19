package frc.team670.robot.commands.auton;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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


/**
 * Starts flush with the edge of B tarmac and shoots preloaded ball
 * Picks up 2 balls along B tarmac edge
 * picks up 2 balls from terminal and comes back to shoot
 * all shooting lower hub
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class BTarmac5BallTerminal extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory, trajectory2;
    private double errorInMeters;
    private Pose2d shootLocation1;

    public BTarmac5BallTerminal(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter) {
        errorInMeters = 0.5;
        shootLocation1 = new Pose2d(5.26, 1.99, Rotation2d.fromDegrees(-148.39));
        trajectory = PathPlanner.loadPath("BTarmac5BallTerminalP1", 2.0, 1);
        //trajectory2 starts and stops at the same spot, shoots from that spot
        trajectory2 = PathPlanner.loadPath("BTarmac5BallTerminalP2", 2.0, 1);
        
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(

            new AutoShootToIntake(conveyor, shooter, intake),
            new ParallelCommandGroup(
                getTrajectoryFollowerCommand(trajectory, driveBase),
                new SequentialCommandGroup(
                    new WaitToShoot(driveBase, shooter, shootLocation1, errorInMeters),
                    new AutoShootToIntake(conveyor, shooter, intake)
                )
            ),
            getTrajectoryFollowerCommand(trajectory, driveBase),
            new AutoShootToIntake(conveyor, shooter, intake),
            getTrajectoryFollowerCommand(trajectory2, driveBase),
            new ShootAllBalls(conveyor, shooter),
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
