package com.stuypulse.robot.commands.oldsuperstructure;

import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure.SuperstructureState;

public class SuperstructurePrepare extends SuperstructureSetState {
    public SuperstructurePrepare() {
        super(SuperstructureState.PREPARING);
    }
}
