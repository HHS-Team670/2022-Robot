package frc.team670.robot.commands.routines.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.conveyor.SetConveyorMode;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;


/**
 * Starts the intake and sets conveyor mode to Intaking
 * @author LakshBhambhani
 */

public class RunIntakeWithConveyor extends ParallelCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public RunIntakeWithConveyor(Intake intake, ConveyorSystem conveyor) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        addCommands(
            new RunIntake(intake),
            new SetConveyorMode(conveyor, ConveyorSystem.Status.INTAKING));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
