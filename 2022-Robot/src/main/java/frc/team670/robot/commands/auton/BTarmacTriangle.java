package frc.team670.robot.commands.auton;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

public class BTarmacTriangle extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Trajectory trajectory;
    private Intake intake;
    private ConveyorSystem conveyor;
    private Shooter shooter;

    public BTarmacTriangle(DriveBase driveBase, Intake intake, Shooter shooter, ConveyorSystem conveyor) {
        //shoot balls then go pick up balls
        trajectory = PathPlanner.loadPath("BTarmacTriangle", 1.0, 0.5);
        // pickup ball
        //trajectory2 = PathPlanner.loadPath("BTarmacTriangleP2", 1.0, 0.5);
        //pickup ball
        //trajectory3 = PathPlanner.loadPath("BTarmacTriangleP3", 1.0, 0.5);
        // shoot balls

        //Logger.consoleLog("Loaded path " + trajectory.toString());
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);

        //Logger.consoleLog("Initial Pose " + trajectory.getStates().get(0).poseMeters);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(
            new AutoShootToIntake(conveyor, shooter, intake),
            getTrajectoryFollowerCommand(trajectory, driveBase),
            new StopDriveBase(driveBase),
            new StopIntake(intake),
            new ShootAllBalls(conveyor, shooter)
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
