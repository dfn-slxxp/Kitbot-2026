package com.stuypulse.robot.subsystems.superstructure.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {

    private DCMotorSim indexerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, 1.0), DCMotor.getCIM(1));

    private double indexerAppliedVolts = 0.0;
    
}
