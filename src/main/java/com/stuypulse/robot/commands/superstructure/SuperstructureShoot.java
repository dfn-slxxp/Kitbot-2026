package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.OldSuperstructure;
import com.stuypulse.robot.subsystems.superstructure.OldSuperstructure.SuperstructureState;

public class SuperstructureShoot extends SuperstructureSetState {
    public SuperstructureShoot() {
        super(SuperstructureState.SHOOTING);
    }
}
