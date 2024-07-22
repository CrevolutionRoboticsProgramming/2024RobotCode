package frc.robot.indexer;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    public static class Settings {
        static final int kSparkId = 24;
        static final int kInitialBeamBreakerId = 1;
        static final int kFinalBreamBreakerID = 2;
        static final double kMaxVoltage = 12.0;
    }

    private static Indexer mInstance;
    private final CANSparkMax mSpark;
    private final DigitalInput mInitialBeamBreaker, mFinalBeamBreaker;

    public Indexer() {
        mSpark = new CANSparkMax(Settings.kSparkId, CANSparkLowLevel.MotorType.kBrushed) {{
            setInverted(true);
        }};
        mInitialBeamBreaker = new DigitalInput(Settings.kInitialBeamBreakerId);
        mFinalBeamBreaker = new DigitalInput(Settings.kFinalBreamBreakerID);

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

    public boolean hasInitialNote() {
        return !mInitialBeamBreaker.get();
    }

    public boolean hasFinalNote() {
        return !mFinalBeamBreaker.get();
    }

   @Override
   public void periodic() {
       SmartDashboard.putBoolean("Indexer Has Note", hasInitialNote());
       SmartDashboard.putBoolean("Indexer Has Note", hasFinalNote());
   }
}
