package frc.robot.shooterflywheel;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterInterpolation extends SubsystemBase{
    // Create an InterpolatingTreeMap to store data points with double keys
    private static InterpolatingTreeMap<Double, Double> dataPercentOutput = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static InterpolatingTreeMap<Double, Double> dataAngle = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static ShooterInterpolation mInstance;
    private double shooterPivotAngle;
    // private static InterpolatingDoubleTreeMap data = new InterpolatingDoubleTreeMap();
    public ShooterInterpolation() {
        // Configures the TreeMap
        configureInterpolatingPercentOutputTreeMap();
        configureInterpolatingAngleTreeMap();
    }

    public static ShooterInterpolation getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterInterpolation();
        }
        return mInstance;
    }

    // Method to set up data values for RPM in the TreeMap
    private void configureInterpolatingPercentOutputTreeMap() {
        // Add data points to the TreeMap
        dataPercentOutput.put(0.5, 0.6);
        dataPercentOutput.put(1.0, 0.6);
        dataPercentOutput.put(1.5, 0.6);
        dataPercentOutput.put(2.0, 0.65);
        dataPercentOutput.put(2.5, 0.7);
        dataPercentOutput.put(3.0, 0.75);
        dataPercentOutput.put(3.5, 0.8);
        dataPercentOutput.put(4.0, 0.85);
        dataPercentOutput.put(4.5, 0.9);
        dataPercentOutput.put(5.0, 0.95);
        dataPercentOutput.put(5.5, 1.0);
    }

    // Method to set up data values for Angles in the TreeMap
    private void configureInterpolatingAngleTreeMap() {
        // Add data points to the TreeMap
        dataAngle.put(0.5, 5.0);
        dataAngle.put(0.9, 5.0);
        dataAngle.put(1.0, 10.0);
        dataAngle.put(1.5, 15.5);
        dataAngle.put(2.0, 21.0);
        dataAngle.put(2.5, 24.0);
        dataAngle.put(3.0, 27.5);
        dataAngle.put(3.5, 28.85);
        dataAngle.put(4.0, 30.35);
        dataAngle.put(4.23, 31.0);
        dataAngle.put(4.5, 32.0);
        dataAngle.put(4.65, 34.8);
        dataAngle.put(4.7, 34.50);
        dataAngle.put(4.829, 32.75);
        dataAngle.put(4.9, 33.50);
        dataAngle.put(4.954, 33.15);
        dataAngle.put(5.1, 33.65);
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
        shooterPivotAngle = dataAngle.get(distance);

        /* Print the interpolated values */
        SmartDashboard.putNumber("Interpolated Shooter Pivot Angle", shooterPivotAngle);

        // Return the interpolated data point
        return shooterPivotAngle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Interpolate Angle]", shooterPivotAngle);
    }
}

