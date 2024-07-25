package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LEDs.LEDs;

public class setLEDColor extends Command{
    private LEDs mLED;

    public setLEDColor() {
        mLED = LEDs.getInstance();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mLED.rainbowMode();
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mLED.setLEDColor(255, 60, 0);
    }
}
