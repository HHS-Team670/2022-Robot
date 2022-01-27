package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
/*
*Gets the distance from target from SmartDashBoard, and predicts the RPM based off that
*/
public class StartShooterByPoseDistance extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private double targetRPM;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public StartShooterByPoseDistance(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        double distanceToTarget = SmartDashboard.getNumber("Vision X", 5);

        Logger.consoleLog("Shooter distance to target %s", distanceToTarget);
        targetRPM = shooter.getTargetRPMForDistance(distanceToTarget);
        Logger.consoleLog("Shooter Stage 2 RPM should be %s", targetRPM);
        shooter.setVelocityTarget(targetRPM);
        shooter.setRampRate(true);
    }

    @Override
    public void execute(){
        shooter.run();
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}