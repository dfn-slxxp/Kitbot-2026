package com.stuypulse.robot.subsystems.superstructure.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

public class IndexerIOReal implements IndexerIO {
    
    private final SparkMax indexMotor;

    private final 

    public IndexerIOReal() {
        this.indexMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
        this.indexMotor.configure(Motors.Superstructure.indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}
