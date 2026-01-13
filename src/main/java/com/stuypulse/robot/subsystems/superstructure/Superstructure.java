package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Superstructure extends SubsystemBase {
    public static final Superstructure instance;

    static {
        instance = new SupertructureImpl();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    public enum SuperstructureState {
        INTAKING(Settings.Superstructure.Intake_Shooter.INTAKE_SPEED, Settings.Superstructure.Indexer.INTAKE_OUTTAKE_SPEED),
        OUTTAKING(Settings.Superstructure.Intake_Shooter.OUTTAKE_SPEED, Settings.Superstructure.Indexer.INTAKE_OUTTAKE_SPEED),
        SHOOTING(Settings.Superstructure.Intake_Shooter.SHOOT_SPEED, Settings.Superstructure.Indexer.SHOOT_SPEED),
        STOP(0, 0);

        private Number main_wheels_speed;
        private Number indexer_speed;

        private SuperstructureState(Number main_wheels_speed, Number indexer_speed) {
            this.main_wheels_speed = main_wheels_speed;
            this.indexer_speed = indexer_speed;
        }

        public double getMainWheelsSpeed() {
            return this.main_wheels_speed.doubleValue();
        }

        public double getIndexerSpeed() {
            return this.indexer_speed.doubleValue();
        }
    }

    protected SuperstructureState state;

    protected Superstructure() {
        this.state = state.STOP;
    }

    public SuperstructureState getState() {
        return state;
    }

    public void setState(SuperstructureState state) {
        this.state = state;
    }
}
