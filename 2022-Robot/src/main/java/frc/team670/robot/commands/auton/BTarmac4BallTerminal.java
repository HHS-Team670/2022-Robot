package frc.team670.robot.commands.auton;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;


/**
 * Starts flush with the edge of B tarmac.
 * Picks up 1 additional ball and shoots both low.
 * Picks up 1 from the ground and 1 from terminal and shoots both low.
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class BTarmac4BallTerminal extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory, trajectory2;

    public BTarmac4BallTerminal(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter) {
        trajectory = PathPlanner.loadPath("BTarmac4BallTerminalP1", 2.0, 1);
        trajectory2 = PathPlanner.loadPath("BTarmac4BallTerminalP2", 2.0, 1);
        
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
            //pickup ball then go back and shoot 2
            new RunIntakeWithConveyor(intake, conveyor),
            getTrajectoryFollowerCommand(trajectory, driveBase),
            new AutoShootToIntake(conveyor, shooter, intake),
            //pick up other ball then pick up terminal ball then go back and shoot 2
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
