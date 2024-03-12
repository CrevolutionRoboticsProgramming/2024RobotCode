package frc.robot.shooterflywheel;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterInterpolation {
    // Create an InterpolatingTreeMap to store data points with double keys
    private static InterpolatingTreeMap<Double, Double> dataPercentOutput = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static InterpolatingTreeMap<Double, Double> dataAngle = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    // private static InterpolatingDoubleTreeMap data = new InterpolatingDoubleTreeMap();
    public ShooterInterpolation() {
        // Configures the TreeMap
        configureInterpolatingPercentOutputTreeMap();
        configureInterpolatingAngleTreeMap();
    }

    // Method to set up data values for RPM in the TreeMap
    private void configureInterpolatingPercentOutputTreeMap() {
        // Add data points to the TreeMap
        dataPercentOutput.put(10.0, 0.2);
        dataPercentOutput.put(20.0, 0.4);
        dataPercentOutput.put(30.0, 0.6);
        dataPercentOutput.put(40.0, 0.8);
        dataPercentOutput.put(50.0, 1.0);
    }

    // Method to set up data values for Angles in the TreeMap
    private void configureInterpolatingAngleTreeMap() {
        // Add data points to the TreeMap
        dataAngle.put(1.0, 5.0);
        dataAngle.put(2.0, 10.0);
        dataAngle.put(3.0, 15.0);
        dataAngle.put(4.0, 20.0);
        dataAngle.put(5.0, 25.0);
        dataAngle.put(6.0, 30.0);
        dataAngle.put(7.0, 35.0);
        dataAngle.put(8.0, 40.0);
        dataAngle.put(9.0, 45.0);
        dataAngle.put(10.0, 50.0);
    }

    // Method to interpolate the data point at a specific distance
    public double getInterpolatedPercentOutput(double distance) {
        /* Get the interpolated entry from the TreeMap for the specified distance */
        double shooterVelocity = dataPercentOutput.get(distance);
        // double shooterRightVelocity = data.get(distance).rightVelocity;
        // double shooterLeftVelocity = data.get(distance).leftVelocity;

        /* Print the interpolated values */
        System.out.println("At distance " + distance);
        System.out.println("Interpolated Shooter Percent Output is " + shooterVelocity);
        // System.out.println("Interpolated left velocity is " + shooterLeftVelocity);
        // System.out.println("Interpolated right velocity is " + shooterRightVelocity);

        // Return the interpolated data point
        return shooterVelocity;
    }

    // Method to interpolate the data point at a specific distance
    public double getInterpolatedAngle(double distance) {
        /* Get the interpolated entry from the TreeMap for the specified distance */
        double shooterPivotAngle = dataAngle.get(distance);

        /* Print the interpolated values */
        System.out.println("At distance " + distance);
        System.out.println("Interpolated shooter pivot angle is " + shooterPivotAngle);

        // Return the interpolated data point
        return shooterPivotAngle;
    }
}

