package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    // This feedforward is in RPS so we convert the desired RPM to RPS when we run the .calculate()
    // method
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

        io.setSidesBrakeMode(false);
        io.setAuxBrakeMode(false);
        io.setIntakeBrakeMode(true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setAmperSpeed(double percent) {
        io.setAmperPercent(percent);
    }

    public void stopAmper() {
        io.setAmperPercent(0);
    }

    public boolean isDetecting() {
        return inputs.noteDetected;
    }

    public void centerNote() {
        io.setLeftPercent(0.08);
        io.setRightPercent(0.08);
    }

    public void fastInSides() {
        io.setLeftPercent(1);
        io.setRightPercent(1);
    }

    public void fastOutSides() {
        io.setLeftPercent(-0.9);
        io.setRightPercent(-0.9);
    }

    public void shooterBackwards() {
        io.setLeftPercent(-1);
        io.setRightPercent(-1);
        io.setAuxPercent(-1);
    }

    public void stopShooterMotors() {
        io.setLeftVoltage(0);
        io.setRightVoltage(0);
        io.setAuxVoltage(0);
    }

    public void stopSideMotors() {
        io.setLeftVoltage(0);
        io.setRightVoltage(0);
    }

    public void ampRackSpeed() {
        io.setLeftPercent(0.25);
        io.setRightPercent(0.25);
        io.setAuxPercent(0);
    }

    public void intake() {
        io.setIntakePercent(+1);
    }

    public void outtake() {
        io.setIntakePercent(-1);
    }

    public void outtakeAmp() {
        io.setIntakePercent(-0.9);
    }

    public void stopIntake() {
        io.setIntakePercent(0);
    }

    public double spinAuxByPid(double rpm) {
        final double rps = rpm / 60;
        final double feedforward = auxFF.calculate(rps);
        final double feedback = auxPID.calculate(0, rps);
        final double volts = feedforward + feedback;
        io.setAuxVoltage(volts);
        return volts;
    }

    public double spinAmperByPid(Rotation2d setpoint) {
        // Add an offset onto the Rot2d and then good subtract it back off to get a value in the range
        // of [-60, +300] instead of [-180, 180]
        final double offset = 120;
        final Rotation2d offset2d = Rotation2d.fromDegrees(offset);
        final double pidOutput =
                amperPID.calculate(
                        inputs.amperPosition.plus(offset2d).getDegrees() - offset,
                        setpoint.plus(offset2d).getDegrees() - offset);
        io.setAmperPercent(pidOutput);
        return pidOutput;
    }
}
