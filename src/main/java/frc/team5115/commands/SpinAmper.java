package frc.team5115.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5115.subsystems.amper.Amper;

public class SpinAmper extends Command {
    private final Amper amper;
    private final Rotation2d setpoint;
    private final double pid_tolerance;
    private double pid;

    public SpinAmper(Amper amper, Rotation2d setpoint) {
        // addRequirements(shooter);
        this.amper = amper;
        this.setpoint = setpoint;
        pid_tolerance = 0.05;
    }

    @Override
    public void execute() {
        pid = amper.spin(setpoint);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pid) < pid_tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        amper.setSpeed(0);
    }
}
