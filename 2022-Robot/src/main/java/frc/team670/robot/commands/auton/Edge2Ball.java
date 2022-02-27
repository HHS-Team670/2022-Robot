package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.deployer.ToggleIntake;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.routines.shoot.WaitToShoot;
import frc.team670.robot.constants.AutonTrajectory;
import frc.team670.robot.constants.HubType;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/**
 * Works for any of the 3 edge2ball paths
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class Edge2Ball extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;
    private Pose2d targetPose;
    private DriveBase driveBase;

    // path names:
    // "ATarmacEdge2Ball"
    // "BTarmacEdgeCenter2Ball"
    // "BTarmacEdgeLower2Ball"
    // Valid hubTypes are "upper" and "lower"
    public Edge2Ball(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter, Deployer deployer, AutonTrajectory pathName, HubType hubType) {
        trajectory = PathPlanner.loadPath(pathName.toString(), 1, 0.5);

        this.driveBase = driveBase;
        
        double errorInMeters = 0.25;
        targetPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        // healthReqs.put(conveyor, HealthState.GREEN);
        // healthReqs.put(intake, HealthState.GREEN);
        // healthReqs.put(shooter, HealthState.GREEN);
        // healthReqs.put(vision, HealthState.GREEN);

        SmartDashboard.putNumber("Auton target x", targetPose.getX());
        SmartDashboard.putNumber("Auton target y", targetPose.getY());
        
        WaitToShoot waitCommand;
        if(hubType == HubType.UPPER)
            waitCommand = new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, -0.93, HubType.UPPER);
        else
            waitCommand = new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, 2.4, HubType.LOWER);

        addCommands(
            new ParallelCommandGroup(
            getTrajectoryFollowerCommand(trajectory, driveBase),
                new SequentialCommandGroup( 
                    // new ParallelCommandGroup(
                    //     new ToggleIntake(deployer),
                    //     new RunIntakeWithConveyor(intake, conveyor)
                    // ),
                    //if doing lower, adjustment should be +2 meters
                    //if doing upper, adjustment should be -0.85 meters
                    // new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, -0.85, "upper"),
                    // new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, 2, "lower"),
                    waitCommand,
                    new ShootAllBalls(conveyor, shooter)
                )
            ),  
            new StopDriveBase(driveBase)
        );
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
