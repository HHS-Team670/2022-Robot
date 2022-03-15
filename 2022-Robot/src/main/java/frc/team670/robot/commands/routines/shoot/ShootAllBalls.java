package frc.team670.robot.commands.routines.shoot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.conveyor.RunConveyor;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Shooter;

public class ShootAllBalls extends SequentialCommandGroup implements MustangCommand {

  private Map<MustangSubsystemBase, HealthState> healthReqs;

  // teleop
  public ShootAllBalls(ConveyorSystem conveyorSystem, Shooter shooter) {      
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
      healthReqs.put(conveyorSystem, HealthState.GREEN);
      healthReqs.put(shooter, HealthState.GREEN);

      addCommands(
        new StartShooter(shooter),  
        new RunConveyor(conveyorSystem, ConveyorSystem.Status.SHOOTING),
        new WaitCommand(shooter.getWaitTime()),
        new StopShooter(shooter)
      );
    }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }

}