package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/**
 * Once the driver aligns the back wheels to the bars under the generator, 
 * drives straight to align with the climbing bar and extends the climber when in position.
 */
public class FullShootVision extends SequentialCommandGroup implements MustangCommand {
    
    private Shooter climber;
    private Vision vis;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public FullShootVision(Shooter c, Vision v) {
        this.climber = c;
        addRequirements(c);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
        addCommands(
                new StartShooterByVisionDistance(climber, vis),
                new Shoot(climber),
                new WaitCommand(RobotConstants.TIME_TO_SHOOT_SECONDS),
                new StopShooter(climber)
            );
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}
