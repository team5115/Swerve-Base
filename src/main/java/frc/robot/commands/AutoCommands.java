package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drivetrain;

public class AutoCommands {
    private AutoCommands() {}

    public static void registerCommands(Drivetrain drivetrain) {
        NamedCommands.registerCommand("InitialShoot", initialShoot());
        NamedCommands.registerCommand("Intake", intake());
        NamedCommands.registerCommand("Shoot", shoot());
        NamedCommands.registerCommand("PrepareClose", prepare(15));
    }

    private static Command initialShoot() {
        return Commands.runOnce(() -> System.out.println("Shooting initial shot"))
                .andThen(Commands.waitSeconds(1));
    }

    private static Command intake() {
        return Commands.runOnce(() -> System.out.println("Intaking")).andThen(Commands.idle());
    }

    private static Command shoot() {
        return Commands.runOnce(() -> System.out.println("Shooting"))
                .andThen(Commands.waitSeconds(0.5));
    }

    private static Command prepare(double angle) {
        return Commands.runOnce(() -> System.out.println("Preparing shot @ " + angle))
                .andThen(Commands.waitSeconds(1.5));
    }
}
