// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5115.commands.AutoCommands;
import frc.team5115.commands.DriveCommands;
import frc.team5115.subsystems.amper.Amper;
import frc.team5115.subsystems.amper.AmperIO;
import frc.team5115.subsystems.amper.AmperIOSim;
import frc.team5115.subsystems.amper.AmperIOSparkMax;
import frc.team5115.subsystems.arm.Arm;
import frc.team5115.subsystems.arm.ArmIO;
import frc.team5115.subsystems.arm.ArmIOSim;
import frc.team5115.subsystems.arm.ArmIOSparkMax;
import frc.team5115.subsystems.drive.Drivetrain;
import frc.team5115.subsystems.drive.GyroIO;
import frc.team5115.subsystems.drive.GyroIONavx;
import frc.team5115.subsystems.drive.ModuleIO;
import frc.team5115.subsystems.drive.ModuleIOSim;
import frc.team5115.subsystems.drive.ModuleIOSparkMax;
import frc.team5115.subsystems.feeder.Feeder;
import frc.team5115.subsystems.feeder.FeederIO;
import frc.team5115.subsystems.feeder.FeederIOSim;
import frc.team5115.subsystems.feeder.FeederIOSparkMax;
import frc.team5115.subsystems.intake.Intake;
import frc.team5115.subsystems.intake.IntakeIO;
import frc.team5115.subsystems.intake.IntakeIOSim;
import frc.team5115.subsystems.intake.IntakeIOSparkMax;
import frc.team5115.subsystems.shooter.Shooter;
import frc.team5115.subsystems.shooter.ShooterIO;
import frc.team5115.subsystems.shooter.ShooterIOSim;
import frc.team5115.subsystems.shooter.ShooterIOSparkMax;
import frc.team5115.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final GyroIO gyro;
    private final Drivetrain drivetrain;
    private final PhotonVision vision;
    private final Arm arm;
    private final Amper amper;
    private final Intake intake;
    private final Feeder feeder;
    private final Shooter shooter;

    // Controller
    private final CommandXboxController joyDrive = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                gyro = new GyroIONavx();
                arm = new Arm(new ArmIOSparkMax());
                amper = new Amper(new AmperIOSparkMax());
                intake = new Intake(new IntakeIOSparkMax());
                feeder = new Feeder(new FeederIOSparkMax());
                shooter = new Shooter(new ShooterIOSparkMax());
                drivetrain =
                        new Drivetrain(
                                gyro,
                                new ModuleIOSparkMax(0),
                                new ModuleIOSparkMax(1),
                                new ModuleIOSparkMax(2),
                                new ModuleIOSparkMax(3));
                vision = new PhotonVision(drivetrain);
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                gyro = new GyroIO() {};
                arm = new Arm(new ArmIOSim());
                amper = new Amper(new AmperIOSim());
                intake = new Intake(new IntakeIOSim());
                feeder = new Feeder(new FeederIOSim());
                shooter = new Shooter(new ShooterIOSim());
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                vision = null;
                break;

            default:
                // Replayed robot, disable IO implementations
                gyro = new GyroIO() {};
                arm = new Arm(new ArmIO() {});
                amper = new Amper(new AmperIO() {});
                intake = new Intake(new IntakeIO() {});
                feeder = new Feeder(new FeederIO() {});
                shooter = new Shooter(new ShooterIO() {});
                drivetrain =
                        new Drivetrain(
                                gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                vision = null;
                break;
        }

        // Register auto commands for pathplanner
        // PhotonVision is passed in here to prevent warnings, i.e. "unused variable: vision"
        AutoCommands.registerCommands(drivetrain, vision, arm, amper, intake, feeder, shooter);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Forward)",
        //         drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Reverse)",
        //         drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Forward)",
        // drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Reverse)",
        // drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // autoChooser.addOption(
        //         "Shooter Aux SysId (Quasistatic Forward)",
        //         shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Shooter Aux SysId (Quasistatic Reverse)",
        //         shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Shooter Aux SysId (Dynamic Forward)",
        //         shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Shooter Aux SysId (Dynamic Reverse)",
        //         shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));

        // joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.start().onTrue(resetFieldOrientation());

        joyDrive
                .back()
                .onTrue(DriveCommands.vomit(intake, feeder, shooter))
                .onFalse(DriveCommands.forceStop(intake, feeder, shooter));

        joyDrive.a().onTrue(DriveCommands.intakeUntilNote(arm, intake, feeder));

        joyDrive
                .y()
                .onTrue(
                        Commands.parallel(arm.stow(), shooter.stop(), feeder.stop(), intake.stop())
                                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        joyDrive
                .b()
                .onTrue(DriveCommands.prepareShoot(arm, intake, feeder, shooter, 15, true))
                .onFalse(
                        feeder
                                .feed()
                                .andThen(shooter.stop())
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        joyDrive
                .x()
                .onTrue(DriveCommands.prepareAmp(arm, amper, intake, feeder))
                .onFalse(DriveCommands.triggerAmp(arm, amper, intake, feeder));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private Command resetFieldOrientation() {
        return Commands.runOnce(
                        () ->
                                drivetrain.setPose(
                                        new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d())),
                        drivetrain)
                .ignoringDisable(true);
    }
}
