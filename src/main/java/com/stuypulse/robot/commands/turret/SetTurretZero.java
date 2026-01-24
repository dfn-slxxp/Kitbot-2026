package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

public class SetTurretZero extends SetTurretState{
        public SetTurretZero() {
        super(TurretState.ZERO);
    }
}
