package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class SetTurretFerry extends SetTurretState {
    public SetTurretFerry() {
        super(TurretState.FERRYING);
    }
}
