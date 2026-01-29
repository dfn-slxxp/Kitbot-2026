package com.stuypulse.robot.commands.oldsuperstructure;

import com.stuypulse.robot.subsystems.oldsuperstructure.Superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WaitUntilAtTargetVelocity extends WaitUntilCommand {
    public WaitUntilAtTargetVelocity() {
        super(() -> Superstructure.getInstance().atTargetVelocity());
    }
}
