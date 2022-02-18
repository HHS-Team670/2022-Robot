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
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;


/**
 * Starts on edge of the A tarmac, picks up 1 additional ball
 * and shoots both to the lower hub.
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class ATarmacEdge2Ball extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;

    public ATarmacEdge2Ball(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter) {
        trajectory = PathPlanner.loadPath("ATarmacEdge2Ball", 1.0, 0.5);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
            // new RunIntakeWithConveyor(intake, conveyor),
            getTrajectoryFollowerCommand(trajectory, driveBase), 
            // new ShootAllBalls(conveyor, shooter),
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
