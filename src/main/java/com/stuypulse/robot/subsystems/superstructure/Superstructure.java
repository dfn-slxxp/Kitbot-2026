package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    
    public enum SuperstructureState {
        OUTTAKING(Settings.Superstructure.Intake_Shooter.OUTTAKE_SPEED, Settings.Superstructure.Indexer.OUTTAKE_SPEED),
        INTAKING(Settings.Superstructure.Intake_Shooter.INTAKE_SPEED, Settings.Superstructure.Indexer.INTAKE_SPEED),
        PREPARING(Settings.Superstructure.Intake_Shooter.SHOOT_SPEED_RPM, 0.0),
        SHOOTING(Settings.Superstructure.Intake_Shooter.SHOOT_SPEED_RPM, Settings.Superstructure.Indexer.OUTTAKE_SPEED),
        STOP(0.0, 0.0);

        private double shooter_speed;
        private double indexer_speed;

        private SuperstructureState(double shooter_speed, double indexer_speed) {
            this.shooter_speed = shooter_speed;
            this.indexer_speed = indexer_speed;
        }

        public double getMainWheelsTargetSpeed() {
            return this.shooter_speed;
        }

        public double getIndexerTargetSpeed() {
            return this.indexer_speed;
        }
    }

    private final SuperstructureIO io;
    

}
