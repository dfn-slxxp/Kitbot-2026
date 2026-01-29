package com.stuypulse.robot.commands.oldsuperstructure;

import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure;
import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure.SuperstructureState;

public class SuperstructureShoot extends SuperstructureSetState {
    public SuperstructureShoot() {
        super(SuperstructureState.SHOOTING);
    }
}
