// package frc.robot.Shooter;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkAbsoluteEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Shooter.ShooterConfig;
// import frc.robot.Robot;
// import frc.robot.CrevoLib.math.Conversions;
// import frc.robot.Elevator.ElevatorConfig;
// import frc.robot.Intake.IntakeConfig;

// public class ShooterPivot extends SubsystemBase {
//     private final CANSparkMax Pivot;

//     private final AbsoluteEncoder encoder;
//     private final DigitalInput LimitSwitch;

//     private ShooterConfig.PivotState state;

//     public ShooterPivot() {
//         Pivot = new CANSparkMax(ShooterConfig.kPivotSparkID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
//         encoder = Pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

//         LimitSwitch = new DigitalInput(ShooterConfig.kHOLimitSwitch);

//         configureMotor();
//         configureSensors();
//     }

//     private void configureMotor() {
//         Pivot.setInverted(ShooterConfig.kPivotMotorInverted);
//         Pivot.setIdleMode(ShooterConfig.kPivotIdleMode);
//         Pivot.setSmartCurrentLimit(ShooterConfig.kDefaultContinuousCurrentLimit, ShooterConfig.kDefaultPeakCurrentLimit);
//     }

//     private void configureSensors() {
//         encoder.setZeroOffset(ShooterConfig.kPivotZeroOffset);
//         encoder.setInverted(ShooterConfig.kPivotEncoderInverted);
//     }

//     public void setIdleMode(CANSparkMax.IdleMode mode) {
//         Pivot.setIdleMode(mode);
//     }

//     public void setOutput(double output) {
//         Pivot.set(output);
//     }

//     public void setCurrentLimit(int continuousLimit, int peakLimit) {
//         Pivot.setSmartCurrentLimit(continuousLimit, peakLimit);
//     }

//     public void stop() {
//         Pivot.set(0);
//     }

//     public ShooterConfig.PivotState getState() {
//         return state;
//     }

//     public void setState(ShooterConfig.PivotState state) {
//         this.state = state;
//     }

//     public boolean getLimitSwitchState() {
//         return !LimitSwitch.get();
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
//         return Math.abs(Pivot.getOutputCurrent());
//     }

//     @Override
//     public void periodic() {
//         if (getLimitSwitchState()) {
//             encoder.setZeroOffset(encoder.getPosition());
//         }
//         System.out.println("[pivot] pos (" + encoder.getPosition() + "), vel (" + getVelocityRps() + ")");
//         System.out.println(getLimitSwitchState());
//     updateSmartDashboard();
//     }

//     private void updateSmartDashboard() {
//         SmartDashboard.putNumber("Pivot Angle (Radians)", getAngleRads());
//         SmartDashboard.putNumber("Pivot Angular Velocity (Radians / Second)", getVelocityRps());
//     }
// }
