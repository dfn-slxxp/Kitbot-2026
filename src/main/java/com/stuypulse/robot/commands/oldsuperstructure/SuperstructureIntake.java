package com.stuypulse.robot.commands.oldsuperstructure;

import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure.SuperstructureState;

public class SuperstructureIntake extends SuperstructureSetState {
    public SuperstructureIntake() {
        super(SuperstructureState.INTAKING);
    }
}
