package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.OldSuperstructure.SuperstructureState;

public class SuperstructureStop extends SuperstructureSetState {
    public SuperstructureStop() {
        super(SuperstructureState.STOP);
    }
}
