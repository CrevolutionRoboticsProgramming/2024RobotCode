// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.ElevatorConfig.ElevatorState;

public class SetPositionElevator extends Command {
    public enum Preset {
        kZero(0.0), kAmp(0.10), kClimb(0.325), kTrap(0.225), kPostTrap(0.18);

        double pos;

        Preset(double pos) {
            this.pos = pos;
        }
    }

    private enum State {
        kInit, kProfile, kDone
    }

    private enum ProfileDirection {
        kDown, kUp
    }

    private final Elevator elevator;
    private final Preset preset;
    private final double kAllowedError = 0.005;
    private final boolean kLoggingEnabled = true;

    private State state;

    private final TrapezoidProfile profile;
    private TrapezoidProfile.State profileInitState, profileGoalState;
    private ProfileDirection profileDirection;
    private long startTs;

    public SetPositionElevator(Preset preset) {
        this.elevator = Elevator.getInstance();
        this.preset = preset;
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            Elevator.Settings.kMaxVelocity,
            Elevator.Settings.kMaxAcceleration
        ));
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        state = State.kInit;
    }

    @Override
    public void execute() {
        switch (state) {
            case kInit:
                startTs = System.currentTimeMillis();
                profileInitState = new TrapezoidProfile.State(elevator.getPosition(), elevator.getVelocity());
                profileGoalState = new TrapezoidProfile.State(preset.pos, 0.0);
                profileDirection = (preset.pos > elevator.getPosition()) ? ProfileDirection.kUp : ProfileDirection.kDown;
                break;
            case kProfile:
                final var request = profile.calculate(getElapsedTime(), profileInitState, profileGoalState);
                elevator.setVelocity(request.velocity, false);
                break;
            case kDone:
                break;
        }
        transitionState();
    }

    private void transitionState() {
        switch (state) {
            case kInit:
                if (isWithinAllowedError(preset.pos)) {
                    changeState(State.kDone, "already at target");
                    break;
                }
                changeState(State.kProfile);
                break;
            case kProfile:
                if (profileDirection == ProfileDirection.kUp && elevator.getUpperLimitState()) {
                    changeState(State.kDone, "hit upper limit");
                    break;
                }
                if (profileDirection == ProfileDirection.kDown && elevator.getLowerLimitState()) {
                    changeState(State.kDone, "hit lower limit");
                }
                if (profile.isFinished(getElapsedTime())) {
                    changeState(State.kDone, "profile complete");
                }
                break;
            case kDone:
                break;
        }
    }

    private boolean isWithinAllowedError(double goalPos) {
        final var pos = elevator.getPosition();
        return Math.abs(goalPos - pos) < kAllowedError;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVelocity(0, false);
        log("end (interrupted = %b)".formatted(interrupted));
    }

    @Override
    public boolean isFinished() {
        return state == State.kDone;
    }

    private double getElapsedTime() {
        return (System.currentTimeMillis() - startTs) / 1000.0;
    }

    private void changeState(State newState) {
        changeState(newState, "");
    }

    private void changeState(State newState, String reason) {
        final var details = new StringBuilder();
        if (state != null) {
            details.append("old state = %s, ".formatted(state.name()));
        }
        details.append("state = %s, ".formatted(newState.name()));
        if (!reason.isEmpty()) {
            details.append("reason = %s".formatted(reason));
        }
        log("%s (%s)".formatted((state == null) ? "initial state" : "change state", details));
        state = newState;
    }

    private void log(String message) {
        if (kLoggingEnabled) {
            System.out.printf("[%s] %s: %s%n", elevator.getClass().getSimpleName(), this.getClass().getSimpleName(), message);
        }
    }
}