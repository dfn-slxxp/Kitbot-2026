package com.stuypulse.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface IndexerIO {
    
    final class IndexerInputs implements LoggableInputs {

        public double motorAppliedVoltage;

        public double motorCurrentDraw;

        public IndexerInputs() {
            this.motorAppliedVoltage = 0.0;
            this.motorCurrentDraw = 0.0;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("motorAppliedVoltage", motorAppliedVoltage);
                SmartDashboard.putNumber("Superstructure/Indexer/motorAppliedVoltage", motorAppliedVoltage);
            table.put("motorCurrentDraw", motorCurrentDraw);
                SmartDashboard.putNumber("Superstructure/Indexer/motorCurrentDraw", motorCurrentDraw);
        }

        @Override
        public void fromLog(LogTable table) {
            motorAppliedVoltage = table.get("motorAppliedVoltage", motorAppliedVoltage);
            motorCurrentDraw = table.get("motorCurrentDraw", motorCurrentDraw);
        }

    }

    void updateInputs(IndexerInputs indexerInputs);

    default void setMotorOutput(double speed) {}

}
