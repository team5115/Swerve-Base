package frc.team5115.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team5115.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SimpleMotorFeedforward feedforward;
    private final PIDController pid;
    private final SysIdRoutine sysID;

    public Shooter(ShooterIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                feedforward = new SimpleMotorFeedforward(0.17484, 0.00223, 0.00030957);
                pid = new PIDController(4.1686E-05, 0, 0);
                break;
            case SIM:
                feedforward = new SimpleMotorFeedforward(0, 0.123, 0.03);
                pid = new PIDController(1, 0, 0);
                break;
            default:
                feedforward = new SimpleMotorFeedforward(0, 0, 0);
                pid = new PIDController(0, 0, 0);
                break;
        }

        pid.setTolerance(20);

        sysID =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public Command stop() {
        return setSpeed(+0);
    }

    public Command setSpeed(double percent) {
        return Commands.runOnce(() -> io.setPercent(percent), this);
    }

    public boolean spinByPid(double rpm) {
        // The feedforward and PID are in RPM
        final double volts = feedforward.calculate(rpm) + pid.calculate(inputs.velocityRPM, rpm);
        io.setVoltage(volts);
        return pid.atSetpoint();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysID.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysID.dynamic(direction);
    }

    public void vomit() {
        io.setPercent(-0.9);
    }

    public void forceStop() {
        io.setPercent(-0.9);
    }
}
