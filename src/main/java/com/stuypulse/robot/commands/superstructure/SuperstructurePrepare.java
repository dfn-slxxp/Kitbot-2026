package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.OldSuperstructure.SuperstructureState;

public class SuperstructurePrepare extends SuperstructureSetState {
    public SuperstructurePrepare() {
        super(SuperstructureState.PREPARING);
    }
}
