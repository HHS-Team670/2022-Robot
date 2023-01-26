package frc.team670.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.LEDs;

public class SetLEDsYellow extends InstantCommand{
    LEDs led;

    public SetLEDsYellow(LEDs led) {
        //led.setColorYellow();
        this.led = led;
    }

    @Override
    public void initialize(){
        led.setColorYellow();
    }
}