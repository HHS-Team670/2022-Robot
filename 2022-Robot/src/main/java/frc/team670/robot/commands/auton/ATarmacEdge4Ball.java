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

//change to ATarmacEdge4Ball for 2022 drivebase
public class ATarmacEdge4Ball extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory, trajectory2, trajectory3, trajectory4, trajectory5, trajectory6;

    public ATarmacEdge4Ball(DriveBase driveBase) {
        //change to ATarmacEdge4Ball for 2022 drivebase
        
        trajectory = PathPlanner.loadPath("ATarmacEdge4BallP1", 2.0, 1);
        //pickup ball
        trajectory2 = PathPlanner.loadPath("ATarmacEdge4BallP2", 2.0, 1);
        //shooting balls
        trajectory3 = PathPlanner.loadPath("ATarmacEdge4BallP3", 2.0, 1);
        //pickup ball
        trajectory4 = PathPlanner.loadPath("ATarmacEdge4BallP4", 2.0, 1);
        //pickup ball
        trajectory5 = PathPlanner.loadPath("ATarmacEdge4BallP5", 2.0, 1);
        //pickup ball
        trajectory6 = PathPlanner.loadPath("ATarmacEdge4BallP6", 2.0, 1);
        //shoot ball
        
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
