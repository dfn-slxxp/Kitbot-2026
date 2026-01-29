package com.stuypulse.robot.commands.oldsuperstructure;

import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure.SuperstructureState;

public class SuperstructureStop extends SuperstructureSetState {
    public SuperstructureStop() {
        super(SuperstructureState.STOP);
    }
}
