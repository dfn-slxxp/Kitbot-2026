package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SuperstructureIntake extends SuperstructureSetState {
    
    private Superstructure superstructure;

    public SuperstructureIntake() {
        super(SuperstructureState.INTAKING);
    }
}
