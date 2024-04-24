package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public Rotation2d armAngle = new Rotation2d();
        public double armVelocityRPM = 0;
        public double armCurrentAmps = 0;
        public double armAppliedVolts = 0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public default void setArmVoltage(double volts) {}
}
