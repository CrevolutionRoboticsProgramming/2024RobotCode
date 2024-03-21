package frc.crevolib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

public abstract class XboxGamepad extends SubsystemBase{

    public boolean configured = false;
    private boolean printed = false;
    public CommandXboxController controller;
    private Rotation2d storedLeftStickDirection = new Rotation2d();
    private Rotation2d storedRightStickDirection = new Rotation2d();

    /**
     * Creates a new Gamepad.
     *
     * @param port The port the gamepad is plugged into
     * @param name The name of the gamepad
     */
    public XboxGamepad(String name, int port) {
        controller = new CommandXboxController(port);
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

    public double getLeftTriggerMagnitude() {
        return (controller.getLeftTriggerAxis() + 1.0) / 2.0;
    }

    public double getRightTriggerMagnitude() {
        return (controller.getRightTriggerAxis() + 1.0) / 2.0;
    }

    /** Setup modifier bumper and trigger buttons */
    public Trigger noBumpers() {
        return controller.rightBumper().negate().and(controller.leftBumper().negate());
    }

    public Trigger leftBumperOnly() {
        return controller.leftBumper().and(controller.rightBumper().negate());
    }

    public Trigger rightBumperOnly() {
        return controller.rightBumper().and(controller.leftBumper().negate());
    }

    public Trigger bothBumpers() {
        return controller.rightBumper().and(controller.leftBumper());
    }

    public Trigger noTriggers() {
        return controller.leftTrigger().negate().and(controller.rightTrigger().negate());
    }

    public Trigger leftTriggerOnly() {
        return controller.leftTrigger().and(controller.rightTrigger().negate());
    }

    public Trigger rightTriggerOnly() {
        return controller.rightTrigger().and(controller.leftTrigger().negate());
    }

    public Trigger bothTriggers() {
        return controller.leftTrigger().and(controller.rightTrigger());
    }

    public Trigger leftYTrigger(Gamepad.ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getLeftY());
    }

    public Trigger leftXTrigger(Gamepad.ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getLeftX());
    }

    public Trigger rightYTrigger(Gamepad.ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getRightY());
    }

    public Trigger rightXTrigger(Gamepad.ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getRightX());
    }

    private Trigger axisTrigger(Gamepad.ThresholdType t, double threshold, DoubleSupplier v) {
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
