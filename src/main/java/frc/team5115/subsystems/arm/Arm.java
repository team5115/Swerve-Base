package frc.team5115.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private final ArmFeedforward armFF;
    private final PIDController armPID;
    private final Rotation2d setpoint;

    public Arm(ArmIO io) {
        this.io = io;
        this.setpoint = new Rotation2d(75);

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                armFF = new ArmFeedforward(0, 0, 0, 0);
                armPID = new PIDController(0, 0.0, 0.0);

                break;
            case SIM:
                armFF = new ArmFeedforward(0, 0, 0, 0);
                armPID = new PIDController(0, 0.0, 0.0);
                break;
            default:
                armFF = new ArmFeedforward(0, 0, 0, 0);
                armPID = new PIDController(0.0, 0.0, 0.0);
                break;
        }
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // Update the pids and feedforward
        final double speed = armPID.calculate(inputs.armAngle.getDegrees(), setpoint.getDegrees());
        double voltage = armFF.calculate(inputs.armAngle.getRadians(), speed);
        voltage = MathUtil.clamp(voltage, -10, +10);

        if (Math.abs(voltage) < 2 * armFF.ks) {
            voltage = 0;
        }

        io.setArmVoltage(voltage);
    }

    public void stop() {
        io.setArmVoltage(0);
    }
}
