package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;

public class LEDs extends LEDSubsystem {

    private Shooter shooter;
    private Intake intake;
    private ConveyorSystem conveyors;
    

    public LEDs(int port, int length, Shooter shooter, Intake intake, ConveyorSystem conveyors) {
        super(port, length);
        this.shooter = shooter;
        this.intake = intake;
        this.conveyors = conveyors;
    }

    @Override
    public void mustangPeriodic() {
        if(!isBlinking) {
            if(shooter.getVelocity() > 0) { // shooter is shooting
                if(shooter.isShooting()) {
                    blink(LEDColor.BLUE);
                    return;
                }
            }

            if(conveyors.getBallCount() > 0) { // show amount of balls in conveyor
                progressBar(LEDColor.GREEN, conveyors.getBallCount() / 2.0);
                return;
            }

            if(intake.isRolling()) { // intake is running
                blink(LEDColor.GREEN);
                return;
            }
        }
        super.mustangPeriodic(); // to handle blinks and setting the led state
    }
    
}
