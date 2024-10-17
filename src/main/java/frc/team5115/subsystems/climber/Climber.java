package frc.team5115.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    // TODO set pid values
    private final PIDController leftPid;
    private final PIDController rightPid;

    public Climber(ClimberIO io) {
        this.io = io;
        
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                leftPid = new PIDController(0.0, 0.0, 0.0);
                rightPid = new PIDController(0.0, 0.0, 0.0);
                break;
            case SIM:
                leftPid = new PIDController(0.5, 0.0, 0.0);
                rightPid = new PIDController(0.5, 0.0, 0.0);
                break;
            default:
                leftPid = new PIDController(0.0, 0.0, 0.0);
                rightPid = new PIDController(0.0, 0.0, 0.0);
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command moveClimberByDelta(double delta) {
        return Commands.runOnce(() -> {
            leftPid.setSetpoint(leftPid.getSetpoint() + delta);
            rightPid.setSetpoint(rightPid.getSetpoint() + delta);
        }, this).andThen(Commands.run(() -> {
            leftPid.calculate(inputs.leftAngle.getDegrees());
            rightPid.calculate(inputs.rightAngle.getDegrees());
        }, this));
    }
}
