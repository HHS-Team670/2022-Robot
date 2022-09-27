package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.ShootThenIntake;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.conveyor.SetConveyorMode;
import frc.team670.robot.constants.AutonTrajectory;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/*
 * BTarmac4BallTerminal2Ball
    * Starts on the edge of B tarmac facing the middle ball.
    * Intakes middle ball, shoots high.
    * Goes to terminal and picks up 2 balls 
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class FourBallPath extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    // private Trajectory trajectory, trajectory2;

    // splitting highhubTerminalP2 into 2 so it can wait at the terminal for 1 sec w
    // WaitCommand
    private Trajectory trajectory, trajectory2, trajectory3;

    private DriveBase driveBase;

    public FourBallPath(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter, Deployer deployer,
            AutonTrajectory pathName) {

        this.driveBase = driveBase;

        if (pathName == AutonTrajectory.BTarmacHighHubTerminal) {
            trajectory = PathPlanner.loadPath("BTarmacHighHubTerminalP1", 2, 1);
            trajectory2 = PathPlanner.loadPath("BTarmacHighHubTerminalP2", 2, 1);
            trajectory3 = PathPlanner.loadPath("BTarmacHighHubTerminalP3", 2, 1, true);
        }

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);

        addCommands(
            new SequentialCommandGroup(
                        // shoot first two balls (one in robot, one in front of tarmac)
                        new ParallelCommandGroup(
                            getTrajectoryFollowerCommand(trajectory, driveBase),
                            new RunIntakeWithConveyor(intake, conveyor),
                            new StartShooter(shooter, 3575)
                        ),
                        new StopDriveBase(driveBase),
                        new ShootThenIntake(conveyor, shooter, intake, 3650), //this one changes!!
                        getTrajectoryFollowerCommand(trajectory2, driveBase),
                        getTrajectoryFollowerCommand(trajectory3, driveBase),
                        new ParallelCommandGroup(
                            new StopDriveBase(driveBase),
                            new StartShooter(shooter, 3650)
                        ),
                        new SetConveyorMode(conveyor, ConveyorSystem.Status.SHOOTING)
            ));
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