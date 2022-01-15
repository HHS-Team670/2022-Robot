package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.paths.ATarmacEdgeBall;

/**
 * Trench Shoot routine for Chezy 2021 (workshop 10/12/2021) google doc link:
 * https://docs.google.com/document/d/1GCqiZlTvnIp7UbRZ-_Gu2sK9tljfNCpqdYkApQ3Qdtk/edit?usp=sharing
 * back of robot on initiation line (closer to trenc)
 * 
 * @author Elise V, justin h, rishabh b
 */
public class Taxi extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Path trajectory;

    public Taxi(DriveBase driveBase) {

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);

        trajectory = new ATarmacEdgeBall(driveBase);

        driveBase.resetOdometry(trajectory.getStartingPose());

        addCommands(
                new ParallelCommandGroup(getTrajectoryFollowerCommand(trajectory, driveBase)));
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