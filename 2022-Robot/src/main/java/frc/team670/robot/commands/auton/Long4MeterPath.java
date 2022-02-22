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
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/**
 * goes 4 meters forwards
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class Long4MeterPath extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;
    private Pose2d targetPose;
    private DriveBase driveBase;

    public Long4MeterPath(DriveBase driveBase, Intake intake, ConveyorSystem conveyor, Shooter shooter) {
        // trajectory = PathPlanner.loadPath("Long4MeterPath", 1, 0.5);
        trajectory = PathPlanner.loadPath("SecondFourMeterPath", 2, 1);
        this.driveBase = driveBase;
        double errorInMeters = 0.25;
        targetPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);

        addCommands(
            //new ParallelCommandGroup(
            getTrajectoryFollowerCommand(trajectory, driveBase),
            //     new SequentialCommandGroup( 
            //         new RunIntakeWithConveyor(intake, conveyor),
            //         //if doing lower, adjustment should be +2 meters
            //         //if doing upper, adjustment should be -1.2 meters
            //         new WaitToShoot(driveBase, shooter, targetPose, errorInMeters, -1.2, "upper"),
            //         new ShootAllBalls(conveyor, shooter) //ADDED VISION
            //     )
            //),  
            new StopDriveBase(driveBase)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        SmartDashboard.putNumber("Auton target x", targetPose.getX());
        SmartDashboard.putNumber("Auton target y", targetPose.getY());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
