
package frc.robot.Elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Elevator.ElevatorConfig.ElevatorState;

public class Elevator extends SubsystemBase {
    private final CANSparkMax spark1;
    private final CANSparkMax spark2;
    private final AbsoluteEncoder encoder;
    private final DigitalInput lowerLimitSwitch, upperLimitSwitch;
                
    private ElevatorState currentState;

    public Elevator(){
        spark1 = new CANSparkMax(ElevatorConfig.kElevatorSparkID1, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        spark2 = new CANSparkMax(ElevatorConfig.kElevatorSparkID2, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        lowerLimitSwitch = new DigitalInput(ElevatorConfig.kLowerLimitSwitchPort);
        upperLimitSwitch = new DigitalInput(ElevatorConfig.kUpperLimitSwitchPort);

        

        //these 2 lines of code need reviewing
        encoder = spark1.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        spark2.follow(spark1);

        configMotors();
        configSensors();
    }

    private void configMotors(){
        spark1.restoreFactoryDefaults();
        spark2.restoreFactoryDefaults();
        spark1.setInverted(ElevatorConfig.kElevatorMotorInverted);
    }

    private void configSensors() {
        encoder.setInverted(ElevatorConfig.kElevatorEncoderInverted);
    }

    public void zero() {
        
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }
    
    public double getPositionMeters() {
        return ElevatorUtils.rotationsToMeters(encoder.getPosition());
    }

    public double getVelocityMps() {
        return ElevatorUtils.rotationsToMeters(encoder.getVelocity() / 60.0);
    }

    public boolean[] getLimitStates() {
        return new boolean[]{!lowerLimitSwitch.get(), !upperLimitSwitch.get()};
    }

    public void setElevatorOutput(double output) {
        final var switchStates = getLimitStates();
        if(switchStates[0] && output < 0) {
            spark1.setVoltage(0);
        }
        else if(switchStates[1] && output > 0) {
            spark1.setVoltage(0);
        }
        else {
            spark1.setVoltage(output);
        }
    }

    public void stop() {
        spark1.set(0);
    }
     @Override
    public void periodic() {
        final var states = getLimitStates();
    //    System.out.println("[Elevator] pos: " + getPositionMeters() + ", vel: " + getVelocityMps());
       // System.out.println("Elevator Encoder Pos " + getEncoderPosition());
        if (getLimitStates()[0]) {
            currentState = ElevatorState.kZero;
            zero();
        }

        updateSmartDashboard();
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Elevator Position (Meters)", getPositionMeters());
        SmartDashboard.putNumber("Elevator Velocity (Meters / Second)", getVelocityMps());
    }
}



