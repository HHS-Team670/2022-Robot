package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;

public class LEDs extends LEDSubsystem {

    private Shooter shooter;
    private Intake intake;
    private ConveyorSystem conveyors;
    private ClimberSystem climbers;

    private boolean isDisabled;

    private LEDColor allianceColor, oppositeAllianceColor;

    public LEDs(int port, int startIndex, int endIndex, Shooter shooter, Intake intake, ConveyorSystem conveyors,
            ClimberSystem climbers) {
        super(port, startIndex, endIndex);
        this.shooter = shooter;
        this.intake = intake;
        this.conveyors = conveyors;
        this.climbers = climbers;
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
        if (isDisabled) {
            rainbow(false);
        } else if (!isBlinking) {
            if (climbers.isRobotClimbing()) {
                blink(LEDColor.PURPLE.dimmer());
            } else if(shooter.foundTarget()){
                blink(LEDColor.GREEN.dimmer());
            }
            else if (conveyors.getBallCount() == 2) { // show amount of balls in conveyor
                solid(oppositeAllianceColor.dimmer());
            } else if (shooter.getVelocity() > 0) { // shooter is shooting
                if (shooter.isShooting()) {
                    blink(oppositeAllianceColor.dimmer(), 10);
                }
            } else if (conveyors.getBallCount() == 1) { // show amount of balls in conveyor
                progressBar(allianceColor.dimmer(), oppositeAllianceColor.dimmer(), 0.5);
            } else if (intake.isRolling()) { // intake is running
                blink(allianceColor.dimmer(), 10);
            } else {
                rainbow(false);
            }
        }
        super.mustangPeriodic(); // to handle blinks and setting the led state
    }

}
