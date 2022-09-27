package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.Shooter;

/**
 * Allows the operator to set a target RPM for the shooter without using vision or ultrasonic.
 * Most often used when vision malfunctions.
 * @author LakshBhambhani
 */
public class OverrideDynamicRPM extends CommandBase implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private MustangController controller;
    private Shooter shooter;

    private boolean justAdvanced = false;

    String overridden_rpm = "NOT OVERIDDEN"; // Debugging field to check which manual rpm override is being used.
                                             // To see in SmartDashboard, uncomment the line in execute()

    public OverrideDynamicRPM(Shooter shooter, MustangController mController) {
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
        this.controller = mController;
        this.shooter = shooter;
    }

    // Called once when the command executes
    @Override
    public void execute() {
        if (!justAdvanced) {

            if (controller.getDPadState() == MustangController.DPadState.RIGHT) {
                shooter.useDynamicSpeed(false);
                shooter.setTargetRPM(1550); // low hub touching the fender
                overridden_rpm = "LOW TOUCHING FENDER";
                justAdvanced = true;
            } else if (controller.getDPadState() == MustangController.DPadState.LEFT) {
                shooter.useDynamicSpeed(false);
                shooter.setTargetRPM(shooter.getDefaultRPM()); // default is set to work for low outside tarmac line
                overridden_rpm = "LOW OUTSIDE TARMAC";
                justAdvanced = true;
            } else if (controller.getDPadState() == MustangController.DPadState.UP) {
                shooter.useDynamicSpeed(false);
                shooter.setTargetRPM(3700); // high hub for right outside tarmac line
                overridden_rpm = "HIGH JUST OUTSIDE TARMAC";
                justAdvanced = true;
            } else if (controller.getDPadState() == MustangController.DPadState.DOWN) {
                shooter.useDynamicSpeed(true);
                overridden_rpm = "NOT OVERRIDED";
                justAdvanced = true;
            }

            SmartDashboard.putString("overridden-rpm", overridden_rpm);

        } else if (controller.getDPadState() == MustangController.DPadState.NEUTRAL) {
            justAdvanced = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}