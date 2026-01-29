package com.stuypulse.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    private final IndexerIO io;
    private final IndexerIO.IndexerInputs inputs;

    private double speed;

    public Indexer(IndexerIO io) {
        this.io = io;
        inputs = new IndexerIO.IndexerInputs();
        this.speed = 0;
    }

    public IndexerIO getIO() {
        return io;
    }

    public void IndexerSetSpeed(double speed) {
        this.speed = speed;
    }

    private void RunIndexerAtSpeeds() {
        io.setMotorOutput(speed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        RunIndexerAtSpeeds();
    }

}