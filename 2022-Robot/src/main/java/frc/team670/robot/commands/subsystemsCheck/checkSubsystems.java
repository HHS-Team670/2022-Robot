package frc.team670.robot.commands.subsystemsCheck;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.conveyor.StopConveyor;
import frc.team670.robot.commands.deployer.ToggleIntake;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.commands.intake.StopIntake;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.subsystems.*;

public class CheckSubsystems extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public CheckSubsystems(ConveyorSystem conveyors, Deployer deployer, DriveBase driveBase, Intake intake, LEDs leds,
            Shooter shooter, TelescopingClimber climber, Vision vision) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(conveyors, HealthState.GREEN);
        healthReqs.put(deployer, HealthState.GREEN);
        healthReqs.put(driveBase, HealthState.GREEN);
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(shooter, HealthState.GREEN);
        healthReqs.put(vision, HealthState.GREEN);

        addCommands(

                new ToggleIntake(deployer),
                new ToggleIntake(deployer),

                new RunIntake(intake),
                new WaitCommand(2),
                new StopIntake(intake),

                new RunConveyor(conveyors, ConveyorSystem.Status.INTAKING),
                new WaitCommand(2),
                new StopConveyor(conveyors),

                new StartShooter(shooter),
                new StopShooter(shooter));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
