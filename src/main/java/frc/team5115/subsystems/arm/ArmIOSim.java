package frc.team5115.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final SingleJointedArmSim arm;
    private double voltage;

    public ArmIOSim() {
        arm =
                new SingleJointedArmSim(
                        DCMotor.getNEO(2), 5.0 * 5.0 * (24.0 / 42.0), 0.0, 1.0, 0.0, 180.0, true, 0.0);
        voltage = 0.0;
    }

    public void updateInputs(ArmIOInputs inputs) {
        arm.update(LOOP_PERIOD_SECS);

        inputs.armAngle = Rotation2d.fromRadians(arm.getAngleRads());
        inputs.armAppliedVolts = voltage;
        inputs.armCurrentAmps = arm.getCurrentDrawAmps();
        inputs.armVelocityRPM = arm.getVelocityRadPerSec() * 60.0;
    }

    public void setArmVoltage(double volts) {
        voltage = MathUtil.clamp(volts, -12.0, 12.0);
        arm.setInputVoltage(voltage);
    }
}
