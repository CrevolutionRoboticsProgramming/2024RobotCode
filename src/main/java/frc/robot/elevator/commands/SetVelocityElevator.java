package frc.robot.elevator.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.Elevator;

import java.util.function.DoubleSupplier;

public class SetVelocityElevator extends Command {
    private final Elevator elevator;
    private final DoubleSupplier velocitySupplier;
    private final boolean isDamped, isOpenLoop;

    private final double kDampThreshold = 0.04;

    SetVelocityElevator(DoubleSupplier velocitySupplier, boolean isDamped, boolean isOpenLoop) {
        elevator = Elevator.getInstance();
        this.velocitySupplier = velocitySupplier;
        this.isDamped = isDamped;
        this.isOpenLoop = isOpenLoop;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        var vel = velocitySupplier.getAsDouble();
        if (elevator.getPosition() > Units.inchesToMeters(28) && vel > 0) {
            vel = 0;
        } else if (elevator.getLowerLimitState() && vel < 0) {
            vel = 0;
        }
        final var scaled = scaleVelocity(vel);
        elevator.setVelocity(scaled, isOpenLoop);
        System.out.printf("vel: %.2f, scaled: %.2f%n", vel, scaled);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVelocity(0, true);
    }

    private double scaleVelocity(double velocity) {
        if (!isDamped) {
            return velocity;
        }
        if (velocity > 0 && (Elevator.Settings.kMaxExtension - elevator.getPosition()) < kDampThreshold) {
            final var scaleFactor = (Elevator.Settings.kMaxExtension - elevator.getPosition() ) / kDampThreshold;
            return velocity * scaleFactor;
        }
        if (velocity < 0 && elevator.getPosition() < kDampThreshold) {
            final var scaleFactor = elevator.getPosition() / kDampThreshold;
            return velocity * scaleFactor;
        }
        return velocity;
    }
}
