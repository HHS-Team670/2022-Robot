package frc.team670.robot.commands.routines.shoot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.conveyor.SetConveyorMode;
import frc.team670.robot.commands.routines.drivebase.AlignAngleToTargetAndShoot;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.commands.shooter.StopShooter;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/**
 * Shoots all the balls in the conveyor
 * @author LakshBhambhani, EliseVambenepe
 */
public class ShootAllBalls extends SequentialCommandGroup implements MustangCommand {

  private Map<MustangSubsystemBase, HealthState> healthReqs;

  public ShootAllBalls(ConveyorSystem conveyorSystem, Shooter shooter) {
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(conveyorSystem, HealthState.GREEN);
    healthReqs.put(shooter, HealthState.GREEN);

    addCommands(
        new StartShooter(shooter),
        new SetConveyorMode(conveyorSystem, "SHOOTING"),
        new WaitCommand(shooter.getWaitTime()),
        new StopShooter(shooter));
  }

  /**
   * Shoots all the balls with a hardcoded RPM
   */
  public ShootAllBalls(ConveyorSystem conveyorSystem, Shooter shooter, double rpm) {
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(conveyorSystem, HealthState.GREEN);
    healthReqs.put(shooter, HealthState.GREEN);

    addCommands(
      new StartShooter(shooter, rpm),
      new SetConveyorMode(conveyorSystem, "SHOOTING"),
        new WaitCommand(shooter.getWaitTime()),
        new StopShooter(shooter));
  }

  /**
   * Shoots all the balls with dynamic RPM (either vision or ultrasonic)
   */
  public ShootAllBalls(ConveyorSystem conveyorSystem, Shooter shooter, DriveBase driveBase, Vision vision) {
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(conveyorSystem, HealthState.GREEN);
    healthReqs.put(shooter, HealthState.GREEN);

    addCommands(
        new AlignAngleToTargetAndShoot(driveBase, vision, conveyorSystem, shooter),
        new WaitCommand(shooter.getWaitTime()),
        new StopShooter(shooter));
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }

}