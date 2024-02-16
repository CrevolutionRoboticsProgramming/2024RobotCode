package frc.robot.LEDs;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Vision.Vision;

public class LED extends SubsystemBase {
    AddressableLED mLEDSubsystem;
    AddressableLEDBuffer mAddressableLEDBuffer;

    Vision mVisionSubsystem;
    Drivetrain mDrivetrainSubsystem;

    public LED(Vision visionSubsystem, Drivetrain drivetrainSubsystem) {
        mLEDSubsystem = new AddressableLED(LEDConfig.LED_PWM_PORT);
        mAddressableLEDBuffer = new AddressableLEDBuffer(LEDConfig.LED_BUFFER_SIZE);

        this.mVisionSubsystem = visionSubsystem;
        this.mDrivetrainSubsystem = drivetrainSubsystem;
        
        mLEDSubsystem.setLength(mAddressableLEDBuffer.getLength());
        mLEDSubsystem.setData(mAddressableLEDBuffer);
        mLEDSubsystem.start();
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()) {
            var alliance = DriverStation.getAlliance();

            if(alliance.equals(Alliance.Red)) {
                setRedAllianceLEDs();
            }
            else {
                setBlueAllianceLEDs();
            }
        }
    }

    private void setRedAllianceLEDs() {
        for(int i = 0; i < mAddressableLEDBuffer.getLength(); i++) {
            mAddressableLEDBuffer.setLED(i, Color.kRed);
        }
    }

    private void setBlueAllianceLEDs() {
        for(int i = 0; i < mAddressableLEDBuffer.getLength(); i++) {
            mAddressableLEDBuffer.setLED(i, Color.kBlue);
        }
    }
}
