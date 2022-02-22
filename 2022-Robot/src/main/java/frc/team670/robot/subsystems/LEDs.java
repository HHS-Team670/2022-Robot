package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        if (!isBlinking) {
            if (conveyors.getBallCount() == 2) { // show amount of balls in conveyor
                solid(LEDColor.BLUE);
                SmartDashboard.putString("LED Status", "Conveyor Solid");
            } else if (shooter.getVelocity() > 0) { // shooter is shooting
                if (shooter.isShooting()) {
                    blink(LEDColor.BLUE.dimmer(), 10);
                    SmartDashboard.putString("LED Status", "Shooter Blink");
                }
            } else if (conveyors.getBallCount() == 1) { // show amount of balls in conveyor
                progressBar(LEDColor.RED,LEDColor.BLUE, 0.5);
                SmartDashboard.putString("LED Status", "Conyeyor Half");
            } else if (intake.isRolling()) { // intake is running
                blink(LEDColor.RED.dimmer(), 10);
                SmartDashboard.putString("LED Status", "Intake Blink");
            }
        }
        super.mustangPeriodic(); // to handle blinks and setting the led state
    }

}
