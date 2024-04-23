package frc.robot.subsystems.drive.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public Rotation2d angle = new Rotation2d();
        public double velocityRadPerMin = 0;
        public double armCurrentAmps = 0;
        public double armAppliedVolts = 0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public default void setArmVoltage(double volts) {}
}
