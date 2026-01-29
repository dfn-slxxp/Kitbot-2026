package com.stuypulse.robot.commands.superstructure.indexer;

import com.stuypulse.robot.subsystems.superstructure.indexer.Indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IndexerSetState extends InstantCommand {
    
    private final Indexer indexer;
    private final double speed;

    public IndexerSetState(Indexer indexer, double speed) {
        this.indexer = indexer;
        this.speed = speed;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.getIO().setMotorOutput(speed);
    }

}