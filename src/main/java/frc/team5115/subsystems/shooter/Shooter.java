package frc.team5115.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SimpleMotorFeedforward auxFF;
    private final PIDController auxPID;
    private final PIDController amperPID;
    private final SysIdRoutine sysID;

    public Shooter(ShooterIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                auxFF = new SimpleMotorFeedforward(0.17484, 0.00223, 0.00030957);
                auxPID = new PIDController(4.1686E-05, 0, 0);
                amperPID = new PIDController(0.004, 0, 0);
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

        sysID =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setAuxVoltage(voltage.baseUnitMagnitude()), null, this));
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
        final double offset = 90.0; // degrees
        final double pidOutput =
                amperPID.calculate(
                        inputs.amperPosition.getDegrees() - offset, setpoint.getDegrees() - offset);
        io.setAmperPercent(pidOutput);
        Logger.recordOutput("Shooter/AmperPidPercent", pidOutput);
        return pidOutput;
    }

    public double spinAuxByPid(double rpm) {
        // The feedforward and PID are in RPM
        final double feedforward = auxFF.calculate(rpm);
        final double feedback = auxPID.calculate(inputs.auxVelocityRPM, rpm);
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysID.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysID.dynamic(direction);
    }
}
