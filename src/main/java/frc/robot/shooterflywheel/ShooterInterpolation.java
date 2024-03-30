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
        dataAngle.put(1.1, 11.1);
        dataAngle.put(1.204, 12.291);
        dataAngle.put(1.303, 13.316);
        dataAngle.put(1.407, 14.4326);
        dataAngle.put(1.5, 15.5);
        dataAngle.put(1.514, 15.6366);
        dataAngle.put(1.603, 16.6598);
        dataAngle.put(1.7065, 17.87686);
        dataAngle.put(1.7974, 18.9238);
        dataAngle.put(1.9035, 19.91);
        dataAngle.put(1.989, 20.9603);
        dataAngle.put(2.0, 21.0);
        dataAngle.put(2.08469, 21.6959);
        dataAngle.put(2.2035, 22.2070);
        dataAngle.put(2.305, 22.7981);
        dataAngle.put(2.3924, 23.4778);
        dataAngle.put(2.5, 26.36);
        dataAngle.put(2.601, 24.75);
        dataAngle.put(2.7136, 28.15);
        dataAngle.put(2.831, 29.06);
        dataAngle.put(2.911, 29.876);
        dataAngle.put(3.004, 30.22);
        dataAngle.put(3.1243, 30.57);
        dataAngle.put(3.184, 30.59);
        dataAngle.put(3.316, 30.61);
        dataAngle.put(3.4236, 30.9001);
        dataAngle.put(3.513, 30.901);
        dataAngle.put(3.63821, 31.5);
        dataAngle.put(3.7027, 31.95);
        dataAngle.put(3.8104, 31.63);
        dataAngle.put(3.894, 32.664);
        dataAngle.put(4.039, 33.01);
        dataAngle.put(4.127, 33.71);
        dataAngle.put(4.2008, 33.35);
        dataAngle.put(4.345, 34.01);
        dataAngle.put(4.384, 33.71);
        dataAngle.put(4.489, 34.435);
        dataAngle.put(4.629, 33.74);
        dataAngle.put(4.756, 34.09);
        dataAngle.put(4.826, 34.417);
        dataAngle.put(4.907, 34.40);
        dataAngle.put(5.0, 34.5);
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

