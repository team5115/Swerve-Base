package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
        final double pidOutput = armPID.calculate(getAngle().getDegrees(), setpoint.getDegrees());
        setTurn(pidOutput, setpoint);
    }

    public void setTurn(double speed, Rotation2d setpoint) {
        if (speed != speed) {
            speed = 0;
        }
        double adjustedRads = getAngle().getRadians(); // + Math.toRadians(30);
        double voltage = MathUtil.clamp(armFF.calculate(adjustedRads, speed), -10, 10);
        // double voltage = MathUtil.clamp(ff.calculate(setpoint.getRadians(-Math.PI), speed), -10, 10);

        if (Math.abs(voltage) < 2 * armFF.ks) {
            voltage = 0;
        }

        io.setArmVoltage(voltage);
    }

    public void stop() {
        io.setArmVoltage(0);
    }

    public Rotation2d getAngle() {
        return inputs.armAngle;
    }

    public double getCurrentAmps() {
        return inputs.armCurrentAmps;
    }
}
