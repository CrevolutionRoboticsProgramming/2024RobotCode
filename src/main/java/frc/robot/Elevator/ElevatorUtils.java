package frc.robot.Elevator;

public class ElevatorUtils {

    public static double rotationsToMeters(double rotations) {
        return ElevatorConfig.kSprocketDiameter * Math.PI * rotations;
    }
}