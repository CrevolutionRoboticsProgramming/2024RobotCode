package frc.robot.intakepivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
    private final CANSparkMax m_Pivot;

    private final AbsoluteEncoder encoder;

    private IntakePivotConfig.PivotState state;

    public IntakePivot() {
        m_Pivot = new CANSparkMax(IntakePivotConfig.kPivotSparkID, MotorType.kBrushless);
        encoder = m_Pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        configureMotor();
        configureSensors();
    }

    public void setOutput(double output) {
        m_Pivot.set(output);
    }

    public void stop() {
        m_Pivot.set(0);
    }

    public void setState(IntakePivotConfig.PivotState state) {
        this.state = state;
    }

    public IntakePivotConfig.PivotState getState() {
        return state;
    }

    public void setCurrentLimit(int continuousLimit, int peakLimit) {
        m_Pivot.setSmartCurrentLimit(continuousLimit, peakLimit);
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        m_Pivot.setIdleMode(mode);
    }

     private void configureMotor() {
        m_Pivot.setInverted(IntakePivotConfig.kPivotMotorInverted);
        m_Pivot.setIdleMode(IntakePivotConfig.kPivotIdleMode);
        m_Pivot.setSmartCurrentLimit(IntakePivotConfig.kDefaultContinuousCurrentLimit, IntakePivotConfig.kDefaultPeakCurrentLimit);
    }

    private void configureSensors() {
        encoder.setZeroOffset(IntakePivotConfig.kPivotZeroOffset);
        encoder.setInverted(IntakePivotConfig.kPivotEncoderInverted);
    }

    /**
     * @return position in radians
     */
    public double getAngleRads() {
        final var angle = Units.rotationsToRadians(encoder.getPosition());
        // return (angle >= 6) ? 0 : angle;
        return angle;
        // return encoder.getPosition();
    }

    /**
     * @return angular velocity in rads / sec
     */
    public double getVelocityRps() {
        return Units.rotationsToRadians(encoder.getVelocity() / 60.0);
    }

    public double getOutputCurrent() {
        return Math.abs(m_Pivot.getOutputCurrent());
    }

    @Override
    public void periodic() {
        System.out.println("[Intake Pivot] pos (" + encoder.getPosition() + "), vel (" + getVelocityRps() + ")");
        updateSmartDashboard();
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Shooter Pivot Angle (Radians)", getAngleRads());
        SmartDashboard.putNumber("Shooter Pivot Angular Velocity (Radians / Second)", getVelocityRps());
    }
}
