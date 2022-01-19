package frc.team670.robot.commands.auton.right;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.path.Path;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.paths.right.RightThroughTrench;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.*;

/**
 * Trench Shoot routine for Chezy 2021 (workshop 10/12/2021) google doc link:
 * https://docs.google.com/document/d/1GCqiZlTvnIp7UbRZ-_Gu2sK9tljfNCpqdYkApQ3Qdtk/edit?usp=sharing
 * back of robot on initiation line
 * @author Elise V, justin h, rishabh b
 */
public class RightShootTrench extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Path trajectory1;

    public RightShootTrench(DriveBase driveBase) {

        // double turretAng = 0;

        this.driveBase = driveBase;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        trajectory1 = new RightThroughTrench(driveBase);
        // turretAng = RobotConstants.rightTurretAng;

        driveBase.resetOdometry(trajectory1.getStartingPose());

        addCommands(
                new ParallelCommandGroup(
                        getTrajectoryFollowerCommand(trajectory1, driveBase))
                

        );
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }
}