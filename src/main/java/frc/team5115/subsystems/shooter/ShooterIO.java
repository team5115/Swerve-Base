package frc.team5115.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double positionRotations = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Run the shooter motor at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Run the shooter motor at the specified percentage. */
    public default void setPercent(double percent) {}
}
