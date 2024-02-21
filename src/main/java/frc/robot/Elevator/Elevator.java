
package frc.robot.Elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Elevator.ElevatorConfig.ElevatorState;
import frc.robot.Shooter.ShooterConfig;

public class Elevator extends SubsystemBase {
    private final CANSparkMax mSpark1;
    private final CANSparkMax mSpark2;
    private final AbsoluteEncoder encoder;
    private final DigitalInput lowerLimitSwitch, upperLimitSwitch;
                
    private ElevatorState currentState;

    public Elevator(){
        mSpark1 = new CANSparkMax(ElevatorConfig.kElevatorSparkID1, MotorType.kBrushless);
        mSpark2 = new CANSparkMax(ElevatorConfig.kElevatorSparkID2, MotorType.kBrushless);
        lowerLimitSwitch = new DigitalInput(ElevatorConfig.kLowerLimitSwitchPort);
        upperLimitSwitch = new DigitalInput(ElevatorConfig.kUpperLimitSwitchPort);

        

        //these 2 lines of code need reviewing
        encoder = mSpark1.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        mSpark2.follow(mSpark1);

        configureMotor();
        configureSensors();
    }

    public void setOutput(double output) {
        final var switchStates = getLimitStates();
        if(switchStates[0] && output < 0) {
            mSpark1.setVoltage(0);
        }
        else if(switchStates[1] && output > 0) {
            mSpark1.setVoltage(0);
        }
        else {
            mSpark1.setVoltage(output);
        }
    }

    public void stop() {
        mSpark1.set(0);
    }

    public void setState(ElevatorState state) {
        currentState = state;
    }

    public ElevatorState getState() {
        return currentState;
    }

    public boolean[] getLimitStates() {
        return new boolean[]{!lowerLimitSwitch.get(), !upperLimitSwitch.get()};
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }
    
    public double getPositionMeters() {
        return ElevatorUtils.rotationsToMeters(encoder.getPosition());
    }

    public double getOutputCurrent() {
        return Math.abs(mSpark1.getOutputCurrent());
    }

    /**
     * @return angular velocity in rads / sec
     */
    public double getVelocityMps() {
        return ElevatorUtils.rotationsToMeters(encoder.getVelocity() / 60.0);
    }

    public void setCurrentLimit(int continuousLimit, int peakLimit) {
        mSpark1.setSmartCurrentLimit(continuousLimit, peakLimit);
        mSpark2.setSmartCurrentLimit(continuousLimit, peakLimit);
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        mSpark1.setIdleMode(mode);
        mSpark2.setIdleMode(mode);
    }

    private void configureMotor() {
        mSpark1.setInverted(ShooterConfig.kPivotMotorInverted);
        mSpark1.setIdleMode(ShooterConfig.kPivotIdleMode);
        mSpark1.setSmartCurrentLimit(ShooterConfig.kDefaultContinuousCurrentLimit, ShooterConfig.kDefaultPeakCurrentLimit);
    }

    private void configureSensors() {
        encoder.setZeroOffset(ShooterConfig.kPivotZeroOffset);
        encoder.setInverted(ShooterConfig.kPivotEncoderInverted);
    }

     @Override
    public void periodic() {
        
        updateSmartDashboard();
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Elevator Position: ", getPositionMeters());
        SmartDashboard.putNumber("Elevator Velocity: ", getVelocityMps());
    }
}



