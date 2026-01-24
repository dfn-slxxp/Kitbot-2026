package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetTurretState extends InstantCommand {
    private final Turret turret;
    private final TurretState targetState;
    
    public SetTurretState(TurretState targetState) {
        turret = Turret.getInstance();
        this.targetState = targetState;
    }

    @Override
    public void initialize() {
        turret.setTurretState(targetState);
    }
}
