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
    private double distance;
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
        dataAngle.put(0.0, 0.0);
        dataAngle.put(1.5, 0.0);
        dataAngle.put(3.89, 29.0);
        dataAngle.put(5.3, 26.5);
        
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
        this.distance = distance;

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
        SmartDashboard.putNumber("Interpolated Passed Dist.", distance);
    }
}

