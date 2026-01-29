package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.OldSuperstructure.SuperstructureState;

public class SuperstructureIntake extends SuperstructureSetState {
    public SuperstructureIntake() {
        super(SuperstructureState.INTAKING);
    }
}
