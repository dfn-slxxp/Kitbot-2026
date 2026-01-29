package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.OldSuperstructure;
import com.stuypulse.robot.subsystems.superstructure.OldSuperstructure.SuperstructureState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperstructureSetState extends InstantCommand {
     
    private final OldSuperstructure superstructure;
    private final SuperstructureState state;

    public SuperstructureSetState(SuperstructureState state) {
        this.superstructure = OldSuperstructure.getInstance();
        this.state = state;

        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setState(state);
    }
}
