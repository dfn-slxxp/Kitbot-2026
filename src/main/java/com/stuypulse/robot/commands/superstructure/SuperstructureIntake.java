package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperstructureIntake extends InstantCommand {
    
    private Superstructure superstructure;

    public SuperstructureIntake() {
        this.superstructure = Superstructure.getInstance();
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setState(SuperstructureState.INTAKING);
    }
}
