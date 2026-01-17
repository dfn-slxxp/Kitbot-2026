package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SuperstructureStop extends SuperstructureSetState {
    
    private Superstructure superstructure;

    public SuperstructureStop() {
        super(SuperstructureState.STOP);
    }
}
