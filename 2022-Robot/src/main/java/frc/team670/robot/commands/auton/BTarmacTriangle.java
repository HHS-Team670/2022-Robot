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
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.routines.shoot.WaitToShoot;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Starts flush with lower hub in B tarmac
 * Shoots lower hub, picks up 2 more, then shoots both into the lower hub
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class BTarmacTriangle extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;
    private Pose2d targetPose;

    public BTarmacTriangle(DriveBase driveBase, Intake intake, Shooter shooter, ConveyorSystem conveyor) {
        //shoot balls then go pick up balls
        trajectory = PathPlanner.loadPath("BTarmacTriangle", 1.0, 0.5);
        double errorInMeters = 0.5;
        targetPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
            new AutoShootToIntake(conveyor, shooter, intake),
            new ParallelCommandGroup(
                getTrajectoryFollowerCommand(trajectory, driveBase),
                
                new SequentialCommandGroup(
                    new WaitToShoot(driveBase, shooter, targetPose, errorInMeters), 
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
