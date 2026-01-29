package com.stuypulse.robot.subsystems.superstructure.indexer;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

public class IndexerIOReal implements IndexerIO {
    
    private final SparkMax indexMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);

    public IndexerIOReal() {
        this.indexMotor.configure(Motors.Superstructure.indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        inputs.motorAppliedVoltage = indexMotor.getAppliedOutput() * 12;
        inputs.motorCurrentDraw = indexMotor.getOutputCurrent();
    }

    @Override
    public void setMotorOutput(double speed) {
        indexMotor.set(speed);
    }

}
