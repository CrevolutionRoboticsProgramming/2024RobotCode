// package frc.robot.Shooter;

// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkAbsoluteEncoder;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ShooterPivot extends SubsystemBase {
//     private final CANSparkMax m_Pivot;

//     private final AbsoluteEncoder encoder;
//     private final DigitalInput LimitSwitch;

//     private ShooterConfig.PivotState state;

//     public ShooterPivot() {
//         m_Pivot = new CANSparkMax(ShooterConfig.kPivotSparkID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
//         encoder = m_Pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

//         LimitSwitch = new DigitalInput(ShooterConfig.kHOLimitSwitch);

//         configureMotor();
//         configureSensors();
//     }

//     public void setOutput(double output) {
//         m_Pivot.set(output);
//     }

//     public void stop() {
//         m_Pivot.set(0);
//     }

//     public boolean getLimitSwitchState() {
//         return !LimitSwitch.get();
//     }

//     public void setState(ShooterConfig.PivotState state) {
//         this.state = state;
//     }

//     public ShooterConfig.PivotState getState() {
//         return state;
//     }

//     public void setCurrentLimit(int continuousLimit, int peakLimit) {
//         m_Pivot.setSmartCurrentLimit(continuousLimit, peakLimit);
//     }

//     public void setIdleMode(CANSparkMax.IdleMode mode) {
//         m_Pivot.setIdleMode(mode);
//     }

//      private void configureMotor() {
//         m_Pivot.setInverted(ShooterConfig.kPivotMotorInverted);
//         m_Pivot.setIdleMode(ShooterConfig.kPivotIdleMode);
//         m_Pivot.setSmartCurrentLimit(ShooterConfig.kDefaultContinuousCurrentLimit, ShooterConfig.kDefaultPeakCurrentLimit);
//     }

//     private void configureSensors() {
//         encoder.setZeroOffset(ShooterConfig.kPivotZeroOffset);
//         encoder.setInverted(ShooterConfig.kPivotEncoderInverted);
//     }

//     /**
//      * @return position in radians
//      */
//     public double getAngleRads() {
//         final var angle = Units.rotationsToRadians(encoder.getPosition());
//         // return (angle >= 6) ? 0 : angle;
//         return angle;
//         // return encoder.getPosition();
//     }

//     /**
//      * @return angular velocity in rads / sec
//      */
//     public double getVelocityRps() {
//         return Units.rotationsToRadians(encoder.getVelocity() / 60.0);
//     }

//     public double getOutputCurrent() {
//         return Math.abs(m_Pivot.getOutputCurrent());
//     }

//     @Override
//     public void periodic() {
//         if (getLimitSwitchState()) {
//             encoder.setZeroOffset(encoder.getPosition());
//         }
//         System.out.println("[Shooter Pivot] pos (" + encoder.getPosition() + "), vel (" + getVelocityRps() + ")");
//         System.out.println(getLimitSwitchState());
//     updateSmartDashboard();
//     }

//     private void updateSmartDashboard() {
//         SmartDashboard.putNumber("Shooter Pivot Angle (Radians)", getAngleRads());
//         SmartDashboard.putNumber("Shooter Pivot Angular Velocity (Radians / Second)", getVelocityRps());
//     }
// }
