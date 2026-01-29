package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.OldSuperstructure.SuperstructureState;

public class SuperstructureOuttake extends SuperstructureSetState {
    public SuperstructureOuttake() {
        super(SuperstructureState.OUTTAKING);
    }
}
