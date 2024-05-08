package frc.team5115.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5115.subsystems.shooter.Shooter;

public class SpinUpShooter extends Command {
    private final Shooter shooter;
    private final double rpm;
    private final boolean neverExit;
    private boolean atSpeed;

    public SpinUpShooter(Shooter shooter, double rpm, boolean neverExit) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.rpm = rpm;
        this.neverExit = neverExit;
    }

    @Override
    public void execute() {
        atSpeed = shooter.spinAuxByPid(rpm);
    }

    @Override
    public boolean isFinished() {
        return atSpeed && !neverExit;
    }
}
