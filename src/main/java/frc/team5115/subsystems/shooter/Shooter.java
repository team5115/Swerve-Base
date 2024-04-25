package frc.team5115.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    // This feedforward is in RPS so we convert the desired RPM to RPS
    // when we run the .calculate() method
    private final SimpleMotorFeedforward auxFF;
    private final PIDController auxPID;
    private final PIDController amperPID;

    public Shooter(ShooterIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                auxFF = new SimpleMotorFeedforward(0.29821, 0.12334, 0.029485);
                auxPID = new PIDController(1, 0, 0);
                amperPID = new PIDController(0.005, 0, 0);
                break;
            case SIM:
                auxFF = new SimpleMotorFeedforward(0, 0.123, 0.03);
                auxPID = new PIDController(1, 0, 0);
                amperPID = new PIDController(0.005, 0, 0);
                break;
            default:
                auxFF = new SimpleMotorFeedforward(0, 0, 0);
                auxPID = new PIDController(0, 0, 0);
                amperPID = new PIDController(0, 0, 0);
                break;
        }

        auxPID.setTolerance(20);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setAmperSpeed(double percent) {
        io.setAmperPercent(percent);
    }

    public Command waitForDetectionState(boolean state, double timeout) {
        return Commands.waitUntil(() -> inputs.noteDetected == state).withTimeout(timeout);
    }

    public Command intake() {
        return setIntakeSpeed(+1);
    }

    public Command outtake() {
        return setIntakeSpeed(-1);
    }

    public Command stopIntake() {
        return setIntakeSpeed(+0);
    }

    public Command stopSides() {
        return setSideSpeeds(+0);
    }

    public Command stopAux() {
        return setAuxSpeed(+0);
    }

    public Command centerNote() {
        return setSideSpeeds(+0.08);
    }

    public Command setSideSpeeds(double percent) {
        return setSideSpeeds(percent, percent);
    }

    public Command setAuxSpeed(double percent) {
        return Commands.runOnce(() -> io.setAuxPercent(percent), this);
    }

    public Command setIntakeSpeed(double percent) {
        return Commands.runOnce(() -> io.setIntakePercent(percent), this);
    }

    public Command setSideSpeeds(double leftPercent, double rightPercent) {
        return Commands.runOnce(
                () -> {
                    io.setLeftPercent(leftPercent);
                    io.setRightPercent(rightPercent);
                },
                this);
    }

    public double spinAmper(Rotation2d setpoint) {
        // Add an offset onto the Rot2d and then good subtract it back off to get a value in
        // the range
        // of [-60, +300] instead of [-180, 180]
        final double offset = 120;
        final Rotation2d offset2d = Rotation2d.fromDegrees(offset);
        final double pidOutput =
                amperPID.calculate(
                        inputs.amperPosition.plus(offset2d).getDegrees() - offset,
                        setpoint.plus(offset2d).getDegrees() - offset);
        io.setAmperPercent(pidOutput);
        Logger.recordOutput("Shooter/AmperPidPercent", pidOutput);
        return pidOutput;
    }

    public double spinAuxByPid(double rpm) {
        final double setpointRPS = rpm / 60.0;
        final double measuredRPS = inputs.auxVelocityRPM / 60.0;
        final double feedforward = auxFF.calculate(setpointRPS);
        final double feedback = auxPID.calculate(measuredRPS, setpointRPS);
        final double volts = feedforward + feedback;
        io.setAuxVoltage(volts);
        return volts;
    }

    /** This command does NOT require the shooter subsystem so that it can override all else */
    public Command vomit() {
        return Commands.runOnce(
                () -> {
                    io.setIntakePercent(-1);
                    io.setLeftPercent(-0.9);
                    io.setRightPercent(-0.9);
                    io.setAuxPercent(-0.9);
                });
    }

    /**
     * This command does NOT require the shooter subsystem. This means it should not be in auto
     * command groups. It also does NOT stop the amper
     */
    public Command stop() {
        return Commands.runOnce(
                () -> {
                    io.setIntakePercent(0);
                    io.setLeftPercent(0);
                    io.setRightPercent(0);
                    io.setAuxPercent(0);
                });
    }
}
