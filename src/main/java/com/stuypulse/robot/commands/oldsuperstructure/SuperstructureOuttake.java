package com.stuypulse.robot.commands.oldsuperstructure;

import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure.SuperstructureState;

public class SuperstructureOuttake extends SuperstructureSetState {
    public SuperstructureOuttake() {
        super(SuperstructureState.OUTTAKING);
    }
}
