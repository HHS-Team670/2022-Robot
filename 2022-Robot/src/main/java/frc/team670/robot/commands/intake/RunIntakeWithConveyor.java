// package frc.team670.robot.commands.intake;

// import java.util.HashMap;
// import java.util.Map;

// import edu.wpi.first.wpilibj2.command.*;
// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
// import frc.team670.robot.constants.RobotConstants;
// import frc.team670.robot.subsystems.*;
// import edu.wpi.first.wpilibj2.command.WaitCommand;


// /**
//  * Runs the intake for the time required to intake one ball? Needs to be checked
//  * @author Sanatan
//  */


// public class RunIntakeWithConveyor extends ParallelCommandGroup implements MustangCommand {

//     private Intake intake;
//     private Conveyors conveyor;
//     private Map<MustangSubsystemBase, HealthState> healthReqs;

//     // Sets up everything

//     public EmptyIntake(Intake intake, Conveyors conveyor) {
//         this.intake = intake;
//         this.conveyor = conveyor;
//         addRequirements(this.intake);
//         healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
//         healthReqs.put(this.intake, HealthState.GREEN);
//         addCommands(
//             new RunIntake(false, this.intake),
//             new RunConveyor(conveyor, Conveyors.Status.INTAKING));
//     }

//     // Returns health state

//     @Override
//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return healthReqs;
//     }

// }
