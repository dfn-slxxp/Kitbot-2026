package com.stuypulse.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class SuperstructureImpl extends Superstructure {
    
    private final TalonFX IntakeShootMotor;
    private final SparkMax IndexerMotor;
    
    public SuperstructureImpl() {
        super();
        
        IntakeShootMotor = new TalonFX(Ports.Superstructure.INTAKE_SHOOTER_MOTOR);
        Motors.Superstructure.intakeShooterMotorConfig.configure(IntakeShootMotor);  
        
        IndexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
        IndexerMotor.configure(Motors.Superstructure.indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setMotorsBasedOnState() {
        IntakeShootMotor.set(state.getMainWheelsSpeed());
        IndexerMotor.set(state.getIndexerSpeed());
    }

    @Override
    public void periodic() {
        setMotorsBasedOnState();
        shooterAtTargetVelocity = (Math.abs(IntakeShootMotor.getVelocity().getValueAsDouble() - state.getMainWheelsSpeed()) <= Settings.Superstructure.Intake_Shooter.SHOOT_TOLERANCE_RPM);
    }
}
