package frc.team670.robot.commands.auton;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.DriveBase;


//split into 2 paths
public class LeftTarmac2Shoot extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory, trajectory2;

    public LeftTarmac2Shoot(DriveBase driveBase) {
        trajectory = PathPlanner.loadPath("LeftTarmac2ShootPt1", 1.0, 0.5);
        trajectory2 = PathPlanner.loadPath("LeftTarmac2ShootPt2", 1.0, 0.5);
        //Logger.consoleLog("Loaded path " + trajectory.toString());
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);

        //Logger.consoleLog("Initial Pose " + trajectory.getStates().get(0).poseMeters);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
            //drive forward
            //intake
            //drive back
            getTrajectoryFollowerCommand(trajectory2, driveBase), 
            new StopDriveBase(driveBase)
            //shoot 2 balls
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
