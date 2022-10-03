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
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.ShootThenIntake;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.constants.AutonTrajectory;
import frc.team670.robot.constants.HubType;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Works for any of the 3 2ball paths
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class TwoBallPath extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory, extension;
    private Pose2d targetPose;
    private DriveBase driveBase;
    private double upperGoalRPM;

    public TwoBallPath(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter, Deployer deployer, AutonTrajectory pathName, HubType hubType) {
        trajectory = PathPlanner.loadPath(pathName.toString(), 1, 0.5);
        extension = PathPlanner.loadPath(pathName.toString() + "Extension", 0.5, 0.25); //BTarmacLower2Ball has no extension as of 6/13/22, so this will be null
        
        //UPDATE THIS!
        if (pathName.equals(AutonTrajectory.ATarmac2Ball)){
            upperGoalRPM = 3570;
        } else if (pathName.equals(AutonTrajectory.BTarmacCenter2Ball)){
            upperGoalRPM = 3570;
        } else if (pathName.equals(AutonTrajectory.BTarmacLower2Ball)){
            upperGoalRPM = 3570;
        }

        this.driveBase = driveBase;
        targetPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);

        //SmartDashboard.putNumber("Auton target x", targetPose.getX());
        //SmartDashboard.putNumber("Auton target y", targetPose.getY());
   
        extension = null; //UNCOMMENT IF YOU WANT TO USE EXTENSION! commented for debug purposes

        if(extension != null) {
            addCommands(
                new ParallelCommandGroup(
                    getTrajectoryFollowerCommand(trajectory, driveBase),
                    new RunIntakeWithConveyor(intake, conveyor),
                    new StartShooter(shooter, upperGoalRPM)
                ), 
                new StopDriveBase(driveBase),
                new ShootThenIntake(conveyor, shooter, intake, upperGoalRPM),
                getTrajectoryFollowerCommand(extension, driveBase), 
                new StopDriveBase(driveBase),
                new ShootAllBalls(conveyor, shooter, 3800)
            );
        } else {
            addCommands(
            new ParallelCommandGroup(
                getTrajectoryFollowerCommand(trajectory, driveBase),
                    new SequentialCommandGroup( 
                        new RunIntakeWithConveyor(intake, conveyor),
                        new StartShooter(shooter, upperGoalRPM)
                    )
                ),
                new ShootAllBalls(conveyor, shooter, upperGoalRPM)
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
