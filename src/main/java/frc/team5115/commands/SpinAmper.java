package frc.team5115.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5115.subsystems.shooter.Shooter;

public class SpinAmper extends Command {
    private final Shooter shooter;
    private final Rotation2d setpoint;
    private final double pid_tolerance;
    private double pid;

    public SpinAmper(Shooter shooter, Rotation2d setpoint) {
        // addRequirements(shooter);
        this.shooter = shooter;
        this.setpoint = setpoint;
        pid_tolerance = 0.05;
    }

    @Override
    public void execute() {
        pid = shooter.spinAmper(setpoint);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pid) < pid_tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAmperSpeed(0);
    }
}
