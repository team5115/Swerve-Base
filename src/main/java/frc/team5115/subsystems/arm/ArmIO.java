package frc.team5115.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d armAngle = new Rotation2d();
        public double armVelocityRPM = 0;
        public double armCurrentAmps = 0;
        public double armAppliedVolts = 0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmVoltage(double volts) {}
}
