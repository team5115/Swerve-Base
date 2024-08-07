package frc.team5115.subsystems.amper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Amper extends SubsystemBase {
    private final AmperIO io;
    private final AmperIOInputsAutoLogged inputs = new AmperIOInputsAutoLogged();
    private final PIDController pid;

    public Amper(AmperIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                pid = new PIDController(0.004, 0, 0);
                break;
            case SIM:
                pid = new PIDController(0.005, 0, 0);
                break;
            default:
                pid = new PIDController(0, 0, 0);
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Amper", inputs);
    }

    public double spin(Rotation2d setpoint) {
        final double offset = 90.0; // degrees
        final double pidOutput =
                pid.calculate(inputs.position.getDegrees() - offset, setpoint.getDegrees() - offset);
        io.setPercent(pidOutput);
        Logger.recordOutput("Shooter/AmperPidPercent", pidOutput);
        return pidOutput;
    }

    public void setSpeed(double percent) {
        io.setPercent(percent);
    }
}
