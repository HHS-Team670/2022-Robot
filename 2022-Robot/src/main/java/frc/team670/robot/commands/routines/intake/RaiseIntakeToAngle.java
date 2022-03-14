package frc.team670.robot.commands.routines.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Deployer;
import frc.team670.robot.subsystems.Intake;

/**
 * Raises the intake to a specified angle
 * 
 * @author AkshatAdsule
 */
public class RaiseIntakeToAngle extends CommandBase implements MustangCommand  {
    private double angle;
    private Deployer deployer;
    private Intake intake;

    private boolean deployed;

    /**
     * Raises the intake to a specified angle
     * @param angle The angle to raise the deployer to
     * @param deployer The deployer
     */
    public RaiseIntakeToAngle(double angle, Deployer deployer, Intake intake) {
        this.angle = angle;
        this.deployer = deployer;
        this.intake = intake;
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
        deployed = deployer.isDeployed();
        if(!deployed){
            deployer.setSystemTargetAngleInDegrees(angle);
        }
        else{
            deployer.setSystemTargetAngleInDegrees(0);
        }
        intake.roll(false);
    }

    @Override
    public boolean isFinished() {
        return deployer.hasReachedTargetPosition();
    }
}
