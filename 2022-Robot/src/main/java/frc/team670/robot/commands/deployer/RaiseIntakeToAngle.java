package frc.team670.robot.commands.deployer;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Deployer;

/**
 * Raises the intake to a specified angle
 * 
 * @author AkshatAdsule
 */
public class RaiseIntakeToAngle extends InstantCommand implements MustangCommand  {
    private double angle;
    private Deployer deployer;

    /**
     * Raises the intake to a specified angle
     * @param angle The angle to raise the deployer to
     * @param deployer The deployer
     */
    public RaiseIntakeToAngle(double angle, Deployer deployer) {
        this.angle = angle;
        this.deployer = deployer;
        addRequirements(deployer);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> requirements = new HashMap<>();
        requirements.put(deployer, HealthState.GREEN);
        return requirements;
    }

    @Override
    public void initialize() {
        deployer.setSystemTargetAngleInDegrees(angle);
    }
}
