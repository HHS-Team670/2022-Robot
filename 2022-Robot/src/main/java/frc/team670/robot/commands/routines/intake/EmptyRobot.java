package frc.team670.robot.commands.routines.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.Intake;


/**
 * Runs the intake for the time required to intake one ball? Needs to be checked
 * @author Sanatan
 */

 
public class EmptyRobot extends ParallelCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    // Sets up everything

    public EmptyRobot(Intake intake, ConveyorSystem conveyor, Deployer deployer) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        addCommands(
            // new ToggleIntake(deployer),
            new RunIntake(intake, true),
            new RunConveyor(conveyor, ConveyorSystem.Status.OUTTAKING));
    }

    // Returns health state

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
