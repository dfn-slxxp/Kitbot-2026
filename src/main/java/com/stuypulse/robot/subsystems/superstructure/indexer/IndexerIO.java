package com.stuypulse.robot.subsystems.superstructure.indexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface IndexerIO {
    
    final class IndexerInputs implements LoggableInputs {

        public boolean motorConnected;

        public double motorSupplyCurrentAmps;

        public double motorOutputVolts;

        public IndexerInputs() {
            this.motorConnected = false;
            this.motorSupplyCurrentAmps = 0.0;
            this.motorOutputVolts = 0.0;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("motorConnected", motorConnected);
                SmartDashboard.putBoolean("Superstructure/Indexer/motorConnected", motorConnected);
            table.put("motorSupplyCurrentAmps", motorSupplyCurrentAmps);
                SmartDashboard.putNumber("Superstructure/Indexer/motorSupplyCurrentAmps", motorSupplyCurrentAmps);
            table.put("motorOutputVolts", motorOutputVolts);
                SmartDashboard.putNumber("Superstructure/Indexer/motorOutputVolts", motorOutputVolts);
        }

        @Override
        public void fromLog(LogTable table) {
            motorConnected = table.get("motorConnected", motorConnected);
            motorSupplyCurrentAmps = table.get("motorSupplyCurrentAmps", motorSupplyCurrentAmps);
            motorOutputVolts = table.get("motorOutputVolts", motorOutputVolts);
        }

    }

    void updateInputs(IndexerInputs indexerInputs);

    default void setMotorOutput(double percent) {}

    default void setMotorBrake(boolean brakeModeEnable) {}

}
