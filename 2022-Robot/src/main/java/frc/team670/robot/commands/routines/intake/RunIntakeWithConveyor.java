package frc.team670.robot.commands.routines.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.*;
import frc.team670.robot.commands.conveyor.*;
import frc.team670.robot.commands.intake.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;


/**
 * Runs the intake and the conveyor at the same time.
 * @author Sanatan, Armaan
 */



// @Laksh PLEASE IGNORE THE NEW CODE
public class RunIntakeWithConveyor extends SequentialCommandGroup implements MustangCommand {

    private Intake intake;
    private ConveyorSystem conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    // Sets up everything

    public RunIntakeWithConveyor(Intake intake, ConveyorSystem conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;
        addRequirements(this.intake);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(this.intake, HealthState.YELLOW);
        healthReqs.put(this.conveyor, HealthState.GREEN);
        addCommands(
            new ParallelCommandGroup(
                new RunIntake(false, this.intake),
                new RunConveyor(this.conveyor, ConveyorSystem.Status.INTAKING)));
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);
        addCommands(
            new RunIntake(intake),
            new RunConveyor(conveyor, ConveyorSystem.Status.INTAKING));
    }

    // Returns health state

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
