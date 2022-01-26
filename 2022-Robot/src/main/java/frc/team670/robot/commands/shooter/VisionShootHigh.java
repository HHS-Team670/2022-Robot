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
public class VisionShootHigh extends SequentialCommandGroup implements MustangCommand {
    
    private Shooter climber;
    private Vision v;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public VisionShootHigh(Shooter climber, Vision vis) {
        this.climber = climber;
        this.v = vis;
        addRequirements(climber);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
        addCommands(
                new ShootBallVision(v, this.climber),
                new Shoot(this.climber),
                new WaitCommand(RobotConstants.TIME_TO_SHOOT_SECONDS),
                new StopShooter(this.climber)
            );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            
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
