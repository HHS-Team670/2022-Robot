package frc.team670.robot.commands.routines.shoot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;


public class AutoShootToIntake extends SequentialCommandGroup implements MustangCommand {


    private Map<MustangSubsystemBase, HealthState> healthReqs;
  
    public AutoShootToIntake(ConveyorSystem conveyorSystem, Shooter shooter, Intake intake, double rpm) {      
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
      healthReqs.put(conveyorSystem, HealthState.GREEN);
      healthReqs.put(shooter, HealthState.GREEN);

      addCommands(
        new ShootAllBalls(conveyorSystem, shooter, rpm),
        new RunIntakeWithConveyor(intake, conveyorSystem)
      );
    } 

    public AutoShootToIntake(ConveyorSystem conveyorSystem, Shooter shooter, Intake intake) {      
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
      healthReqs.put(conveyorSystem, HealthState.GREEN);
      healthReqs.put(shooter, HealthState.GREEN);

      addCommands(
        new ShootAllBalls(conveyorSystem, shooter),
        new RunIntakeWithConveyor(intake, conveyorSystem)
      );
    } 

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
      return healthReqs;
    }
}