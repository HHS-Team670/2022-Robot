package frc.team670.robot.commands.auton;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.conveyor.StopConveyor;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.routines.shoot.AutoShootToIntake;
import frc.team670.robot.commands.routines.shoot.ShootAllBalls;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

/**
 * Starts on the edge of the A tarmac with 1 ball. Scores a total of 4 balls.
 * https://miro.com/app/board/uXjVOWE2OxQ=/
 */
public class ATarmacEdge4Ball extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Intake intake;
    private ConveyorSystem conveyor;
    private Shooter shooter;
    private Trajectory trajectory, trajectory2;

    public ATarmacEdge4Ball(DriveBase driveBase, Intake intake, Shooter shooter, ConveyorSystem conveyor) {
        //change to ATarmacEdge4Ball for 2022 drivebase
        this.intake = intake;
        this.shooter = shooter;
        this.conveyor = conveyor;
        
        trajectory = PathPlanner.loadPath("ATarmacEdge4BallP1", 2.0, 1);
        trajectory2 = PathPlanner.loadPath("ATarmacEdge4BallP2", 2.0, 1);
        
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);

        driveBase.resetOdometry(trajectory.getStates().get(0).poseMeters);
        addCommands(

            new RunIntakeWithConveyor(intake, conveyor),

            getTrajectoryFollowerCommand(trajectory, driveBase),

            new ShootAllBalls(conveyor, shooter),

            getTrajectoryFollowerCommand(trajectory2, driveBase),

            new ShootAllBalls(conveyor, shooter),

            new ParallelCommandGroup(
                new StopIntake(intake),
                new StopConveyor(conveyor),
                new StopDriveBase(driveBase)
            )
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
