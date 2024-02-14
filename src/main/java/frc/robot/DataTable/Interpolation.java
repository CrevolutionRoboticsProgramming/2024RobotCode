package frc.robot.DataTable;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

// Define a class to represent a data point with shooter pivot angle, left velocity, and right velocity
class DataPoint {
    double shooterPivotAngle; // Shooter pivot angle variable
    double leftVelocity;      // Left velocity variable
    double rightVelocity;     // Right velocity variable

    // Constructor to initialize the data point with shooter pivot angle, left velocity, and right velocity
    public DataPoint(double shooterPivotAngle, double leftVelocity, double rightVelocity) {
        this.shooterPivotAngle = shooterPivotAngle;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
    }
}

// Main class for the interpolation example
public class Interpolation {
    // Create an InterpolatingTreeMap to store data points with double keys
    private static InterpolatingTreeMap<Double, DataPoint> data = new InterpolatingTreeMap<Double, DataPoint>(null, null);

    // Main method where the program starts execution
    public static void main(String[] args) {
        // Add sample data points to the TreeMap
        data.put(10.0, new DataPoint(45, 20, 22));
        data.put(20.0, new DataPoint(30, 25, 27));
        data.put(30.0, new DataPoint(60, 30, 32));
        data.put(40.0, new DataPoint(45, 35, 37));
        data.put(50.0, new DataPoint(90, 40, 42));
    }

    // Method to interpolate the data point at a specific distance
    public static DataPoint interpolate(double distance) {

    // Get the interpolated entry from the TreeMap for the specified distance
        double shooterPivotAngle = data.get(distance).shooterPivotAngle;
        double shooterRightVelocity = data.get(distance).rightVelocity;
        double shooterLeftVelocity = data.get(distance).leftVelocity;

        DataPoint InterpolatedValues = new DataPoint(shooterPivotAngle, shooterLeftVelocity, shooterRightVelocity);

        // Print the interpolated values
        System.out.println("At distance " + distance);
        System.out.println("Interpolated shooter pivot angle is " + shooterPivotAngle);
        System.out.println("Interpolated left velocity is " + shooterLeftVelocity);
        System.out.println("Interpolated right velocity is " + shooterRightVelocity);

    // Return the interpolated data point
        return InterpolatedValues;
    }
}