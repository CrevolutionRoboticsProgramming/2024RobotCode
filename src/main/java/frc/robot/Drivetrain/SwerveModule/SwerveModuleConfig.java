package frc.robot.Drivetrain.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {
    public final int driveMotorID, angleMotorID, cancoderID;
    public final Rotation2d angleOffset;

    public SwerveModuleConfig(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
