package frc.team670.robot.commands.auton;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class ATarmacEdge2Ball extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;

    public BTarmacFlushed2InnerOuterBall(DriveBase driveBase) {
        trajectory = PathPlanner.loadPath("BTarmacFlushed2InnerOuterBall", 1.0, 0.5);
        //Logger.consoleLog("Loaded path " + trajectory.toString());
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);

        //Logger.consoleLog("Initial Pose " + trajectory.getStates().get(0).poseMeters);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
            getTrajectoryFollowerCommand(trajectory, driveBase)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return healthReqs;
    }

}
