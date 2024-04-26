package frc.team5115.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leftVelocityRPM = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public double rightVelocityRPM = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;

        public double auxVelocityRPM = 0.0;
        public double auxAppliedVolts = 0.0;
        public double auxCurrentAmps = 0.0;

        public double intakeVelocityRPM = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;

        public Rotation2d amperPosition = new Rotation2d();
        public double amperAppliedVolts = 0.0;
        public double amperCurrentAmps = 0.0;

        public boolean noteDetected = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Run the left motor at the specified voltage. */
    public default void setLeftVoltage(double volts) {}

    /** Run the left motor at the specified percentage. */
    public default void setLeftPercent(double percent) {}

    /** Run the right motor at the specified voltage. */
    public default void setRightVoltage(double volts) {}

    /** Run the right motor at the specified percentage. */
    public default void setRightPercent(double percent) {}

    /** Run the aux motor at the specified voltage. */
    public default void setAuxVoltage(double volts) {}

    /** Run the aux motor at the specified percentage. */
    public default void setAuxPercent(double percent) {}

    /** Run the intake motor at the specified percentage. */
    public default void setIntakePercent(double percent) {}

    /** Run the intake motor at the specified voltage. */
    public default void setAmperVoltage(double volts) {}

    /** Run the amper motor at the specified percentage. */
    public default void setAmperPercent(double percent) {}

    /** Run the intake motor at the specified voltage. */
    public default void setIntakeVoltage(double volts) {}
}
