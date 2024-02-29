package frc.robot.indexer;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    static class Settings {
        static final int kSparkId = 25;
        static final int kBeamBreakerId = 0;
        static final double kMaxVoltage = 12.0;
    }

    private static Indexer mInstance;
    private final CANSparkMax mSpark;
    private final DigitalInput mBeamBreaker;

    public Indexer() {
        mSpark = new CANSparkMax(Settings.kSparkId, CANSparkLowLevel.MotorType.kBrushed);
        mBeamBreaker = new DigitalInput(Settings.kBeamBreakerId);
    }

    public static Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public void setOutput(double percentOut) {
        mSpark.setVoltage(percentOut * Settings.kMaxVoltage);
    }

    public boolean hasNote() {
        return !mBeamBreaker.get();
    }

}
