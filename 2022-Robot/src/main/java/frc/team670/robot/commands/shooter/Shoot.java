package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Once the driver aligns the back wheels to the bars under the generator, 
 * drives straight to align with the climbing bar and extends the climber when in position.
 */
public class Shoot extends SequentialCommandGroup implements MustangCommand {
    
    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private final double RPM = 408.0; // change this later

    public Shoot(Shooter s) {
        this.shooter = s;
        addRequirements(s);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(s, HealthState.GREEN);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            addCommands(
                new SetRPMTarget(RPM, shooter),
                new ShootBall(shooter),
                new WaitCommand(RobotConstants.TIME_TO_SHOOT_SECONDS),
                new StopShooter(shooter)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}
