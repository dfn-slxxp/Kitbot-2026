package com.stuypulse.robot.commands.oldsuperstructure;

import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure;
import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure.SuperstructureState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperstructureSetState extends InstantCommand {
     
    private final Superstructure superstructure;
    private final SuperstructureState state;

    public SuperstructureSetState(SuperstructureState state) {
        this.superstructure = Superstructure.getInstance();
        this.state = state;

        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setState(state);
    }
}
