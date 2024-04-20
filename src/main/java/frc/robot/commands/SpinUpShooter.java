package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SpinUpShooter extends Command {
    private final Shooter shooter;
    private final double pid_tolerance;
    private final double rpm;
    private final boolean neverExit;
    private boolean atSpeed;

    public SpinUpShooter(Shooter shooter, double rpm, boolean neverExit) {
        this.shooter = shooter;
        this.rpm = rpm;
        this.neverExit = neverExit;
        pid_tolerance = 10;
    }

    @Override
    public void execute() {
        double pid = shooter.spinAuxByPid(rpm * 1);
        atSpeed = Math.abs(pid) < pid_tolerance;
    }

    @Override
    public boolean isFinished() {
        return atSpeed && !neverExit;
    }
}
