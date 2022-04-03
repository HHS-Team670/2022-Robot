package frc.team670.robot.commands.auton;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.deployer.ToggleIntake;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.commands.routines.shoot.SetTargetRPM;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.routines.shoot.WaitToShoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.constants.AutonTrajectory;
import frc.team670.robot.constants.HubType;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Works for any of the 3 edge2ball paths
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class Edge2Ball extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory, extension;
    private Pose2d targetPose;
    private DriveBase driveBase;
    private Shooter shooter;
    private double upperGoalRPM = shooter.getDefaultRPM(); //UPDATE THIS FOR RPM!!


    // path names:
    // "ATarmacEdge2Ball" \ "ATarmacEdge2BallExtension"
    // "BTarmacEdgeCenter2Ball" \ "BTarmacEdgeCenter2BallExtension"
    // "BTarmacEdgeLower2Ball"
    // Valid hubTypes are "upper" and "lower"
    public Edge2Ball(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter, Deployer deployer, AutonTrajectory pathName, HubType hubType) {
        trajectory = PathPlanner.loadPath(pathName.toString(), 1, 0.5);
        extension = PathPlanner.loadPath(pathName.toString() + "Extension", 0.5, 0.25);

        //UPDATE THIS!
        if (pathName.equals(AutonTrajectory.ATarmacEdge2Ball)){
            upperGoalRPM = 30;
        } else if (pathName.equals(AutonTrajectory.BTarmacEdgeCenter2Ball)){
            upperGoalRPM = 30;
        } else if (pathName.equals(AutonTrajectory.BTarmacEdgeLower2Ball)){
            upperGoalRPM = 30;
        }

        this.shooter = shooter;
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
        
        // WaitToShoot waitCommand;
        // if(hubType == HubType.UPPER)
            // waitCommand = new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, -1.2, HubType.UPPER);

        
        if(extension != null) {
            addCommands(
                new ParallelCommandGroup(
                getTrajectoryFollowerCommand(trajectory, driveBase),
                    new SequentialCommandGroup( 
                        // new ParallelCommandGroup(
                            new RunIntakeWithConveyor(intake, conveyor),
                        // ),
                        // new SetTargetRPM(shooter, upperGoalRPM),
                        // new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, upperGoalRPM),
                        new StartShooter(shooter, upperGoalRPM),
                        new AutoShootToIntake(conveyor, shooter, intake, upperGoalRPM)
                    )
                ), 
                getTrajectoryFollowerCommand(extension, driveBase), 
                new ShootAllBalls(conveyor, shooter, upperGoalRPM),
                new StopDriveBase(driveBase)
            );
        } else {
            addCommands(
            new ParallelCommandGroup(
                getTrajectoryFollowerCommand(trajectory, driveBase),
                    new SequentialCommandGroup( 
                        // new ParallelCommandGroup(
                            new RunIntakeWithConveyor(intake, conveyor),
                        // ),
                        // new SetTargetRPM(shooter, upperGoalRPM),
                        // new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, upperGoalRPM),
                        new StartShooter(shooter, upperGoalRPM),

                        new ShootAllBalls(conveyor, shooter, upperGoalRPM)
                    )
                )
            );
        }
        
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
