package frc.crevolib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

public abstract class Gamepad extends SubsystemBase {

    public boolean configured = false;
    private boolean printed = false;
    public CommandPS5Controller controller;
    private Rotation2d storedLeftStickDirection = new Rotation2d();
    private Rotation2d storedRightStickDirection = new Rotation2d();

    /**
     * Creates a new Gamepad.
     *
     * @param port The port the gamepad is plugged into
     * @param name The name of the gamepad
     */
    public Gamepad(String name, int port) {
        controller = new CommandPS5Controller(port);
    }

    @Override
    public void periodic() {
        configure();
    }

    // Configure the driver controller
    public void configure() {
        // Detect whether the xbox controller has been plugged in after start-up
        if (!configured) {
            boolean isConnected = controller.getHID().isConnected();
            if (!isConnected) {
                if (!printed) {
                    System.out.println("[error] " + getName() + ": gamepad not connected");
                    printed = true;
                }
                return;
            }

            // Configure button bindings once the driver controller is connected
            if (DriverStation.isTest()) {
                setupTestButtons();
            } else if (DriverStation.isDisabled()) {
                setupDisabledButtons();
            } else {
                setupTeleopButtons();
            }
            configured = true;

            System.out.println("[info] " + getName() + ": gamepad connected");
        }
    }

    // Reset the controller configure, should be used with
    // CommandScheduler.getInstance.clearButtons()
    // to reset buttons
    public void resetConfig() {
        configured = false;
        configure();
    }

    /* Zero is stick up, 90 is stick to the left*/
    public Rotation2d getLeftStickDirection() {
        double x = -1 * controller.getLeftX();
        double y = -1 * controller.getLeftY();
        if (x != 0 || y != 0) {
            storedLeftStickDirection = new Rotation2d(y, x);
        }
        return storedLeftStickDirection;
    }

    public double getLeftStickCardinals() {
        double stickAngle = getLeftStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getLeftStickMagnitude() {
        double x = -1 * controller.getLeftX();
        double y = -1 * controller.getLeftY();
        return Math.sqrt(x * x + y * y);
    }

    public Rotation2d getRightStickDirection() {
        double x = controller.getRightX();
        double y = controller.getRightY();
        if (x != 0 || y != 0) {
            storedRightStickDirection = new Rotation2d(y, x);
        }
        return storedRightStickDirection;
    }

    public double getRightStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getRightStickMagnitude() {
        double x = controller.getRightX();
        double y = controller.getRightY();
        return Math.sqrt(x * x + y * y);
    }

    /** Setup modifier bumper and trigger buttons */
    public Trigger noBumpers() {
        return controller.R1().negate().and(controller.R1().negate());
    }

    public Trigger leftBumperOnly() {
        return controller.L1().and(controller.R1().negate());
    }

    public Trigger rightBumperOnly() {
        return controller.R1().and(controller.L1().negate());
    }

    public Trigger bothBumpers() {
        return controller.R1().and(controller.L1());
    }

    public Trigger noTriggers() {
        return controller.L2().negate().and(controller.R2().negate());
    }

    public Trigger leftTriggerOnly() {
        return controller.L2().and(controller.R2().negate());
    }

    public Trigger rightTriggerOnly() {
        return controller.R2().and(controller.L2().negate());
    }

    public Trigger bothTriggers() {
        return controller.L2().and(controller.R2());
    }

    public Trigger leftYTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getLeftY());
    }

    public Trigger leftXTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getLeftX());
    }

    public Trigger rightYTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getRightY());
    }

    public Trigger rightXTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getRightX());
    }

    private Trigger axisTrigger(ThresholdType t, double threshold, DoubleSupplier v) {
        return new Trigger(() -> {
            double value = v.getAsDouble();
            return switch (t) {
                case GREATER_THAN -> value > threshold;
                case LESS_THAN -> value < threshold;
                // i.e. deadband
                case ABS_GREATER_THAN ->
                        Math.abs(value) > threshold;
            };
        });
    }

    public enum ThresholdType {
        GREATER_THAN,
        LESS_THAN,
        ABS_GREATER_THAN;
    }

    public abstract void setupTeleopButtons();

    public abstract void setupDisabledButtons();

    public abstract void setupTestButtons();
}
