package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.indexer.commands.LoadNote.Profile;

import java.util.function.DoubleSupplier;

public class IndexerCommands {
    public static Command setOutput(DoubleSupplier outputSupplier) {
        return new SetOutputIndexer(outputSupplier);
    }

    public static Command grabNote() {
        return new LoadNote(LoadNote.Profile.kLowLoad);
    }

    public static Command unJamNote() {
        return new GrabNote(GrabNote.Profile.kHighLoad);
    }

    // public static Command grabNote() {
    //     return new GrabNote(GrabNote.Profile.kLowLoad);
    // }
}
