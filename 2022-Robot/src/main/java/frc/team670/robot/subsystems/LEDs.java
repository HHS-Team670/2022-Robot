package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;

public class LEDs extends LEDSubsystem {

    private Shooter shooter;
    private Intake intake;
    private ConveyorSystem conveyors;
    private boolean isDisabled;

    private LEDColor allianceColor, oppositeAllianceColor;    

    public LEDs(int port, int length, Shooter shooter, Intake intake, ConveyorSystem conveyors) {
        super(port, length);
        this.shooter = shooter;
        this.intake = intake;
        this.conveyors = conveyors;
        this.isDisabled = true;
    }

    public void setIsDisabled(boolean isDisabled) {
        this.isDisabled = isDisabled;
    }

    public void setAllianceColors(LEDColor alliance, LEDColor oppositeAlliance) {
        this.allianceColor = alliance;
        this.oppositeAllianceColor = oppositeAlliance;
    }

    @Override
    public void mustangPeriodic() {
        if(isDisabled) {
            rainbow();
        } else if (!isBlinking) {
            if (conveyors.getBallCount() == 2) { // show amount of balls in conveyor
                solid(allianceColor);
            } else if (shooter.getVelocity() > 0) { // shooter is shooting
                if (shooter.isShooting()) {
                    blink(allianceColor.dimmer(), 10);
                }
            } else if (conveyors.getBallCount() == 1) { // show amount of balls in conveyor
                progressBar(oppositeAllianceColor, allianceColor, 0.5);                
            } else if (intake.isRolling()) { // intake is running
                blink(oppositeAllianceColor.dimmer(), 10);
            } else {
                rainbow();
            }
        }
        super.mustangPeriodic(); // to handle blinks and setting the led state
    }

}
