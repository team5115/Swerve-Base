package frc.team5115;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final CommandXboxController joyManip = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Shuffleboard
    private final GenericEntry noteDetectedEntry;

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
                // vision = null;
                noteDetectedEntry =
                        Shuffleboard.getTab("SmartDashboard").add("Has note?", false).getEntry();
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
                noteDetectedEntry = null;
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
                noteDetectedEntry = null;
                break;
        }

        // Register auto commands for pathplanner
        // PhotonVision is passed in here to prevent warnings, i.e. "unused variable: vision"
        registerCommands(drivetrain, vision, arm, amper, intake, feeder, shooter);

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
        //         drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Dynamic Reverse)",
        //         drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Shooter SysId (Quasistatic Forward)",
        //         shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Shooter SysId (Quasistatic Reverse)",
        //         shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Shooter SysId (Dynamic Forward)",
        //         shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Shooter SysId (Dynamic Reverse)",
        //         shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drivetrain,
                        () -> -joyDrive.getLeftY(),
                        () -> -joyDrive.getLeftX(),
                        () -> -joyDrive.getRightX()));

        // joyDrive.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        joyDrive.start().onTrue(resetRobotPose());

        joyManip
                .back()
                .onTrue(DriveCommands.vomit(intake, feeder, shooter))
                .onFalse(DriveCommands.forceStop(intake, feeder, shooter));

        joyManip.a().onTrue(DriveCommands.intakeUntilNote(arm, intake, feeder));

        joyManip
                .y()
                .onTrue(
                        Commands.parallel(arm.stow(), shooter.stop(), feeder.stop(), intake.stop())
                                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        joyManip
                .b()
                .onTrue(DriveCommands.prepareShoot(arm, intake, feeder, shooter, 15, 5000))
                .onFalse(
                        DriveCommands.feed(intake, feeder)
                                .andThen(shooter.stop())
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        joyManip
                .x()
                .onTrue(DriveCommands.prepareAmp(arm, amper, intake, feeder))
                .onFalse(DriveCommands.triggerAmp(arm, amper, intake, feeder));

        joyManip
                .leftBumper()
                .onTrue(
                        DriveCommands.automaticallyPrepareShoot(drivetrain, arm, intake, feeder, shooter)
                                .andThen(DriveCommands.feed(intake, feeder), shooter.stop()));
    }

    public void robotPeriodic() {
        if (noteDetectedEntry != null) {
            noteDetectedEntry.setBoolean(feeder.noteDetected());
        }
    }

    /**
     * Registers commands for pathplanner to use in autos
     *
     * @param shooter the shooter subsystem
     * @param arm the arm subsystem
     * @param drivetrain the drivetrain subsytem (not currently used)
     * @param photonVision the photonvision subsystem (not currently used)
     */
    public static void registerCommands(
            Drivetrain drivetrain,
            PhotonVision vision,
            Arm arm,
            Amper amper,
            Intake intake,
            Feeder feeder,
            Shooter shooter) {
        // Bring the arm down, turn the intake and shooter on, and then feed first shot
        NamedCommands.registerCommand(
                "InitialShot",
                Commands.parallel(
                                arm.setAngle(Rotation2d.fromDegrees(15)),
                                intake.setSpeed(+1),
                                shooter.spinToSpeed(5000))
                        .andThen(feeder.setSpeeds(+1)));

        NamedCommands.registerCommand(
                "Intake",
                Commands.sequence(
                                arm.setAngle(Rotation2d.fromDegrees(0)),
                                feeder.centerNote(),
                                feeder.waitForDetectionState(true, 1.5),
                                Commands.waitSeconds(0.25),
                                feeder.stop())
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        NamedCommands.registerCommand(
                "Feed", Commands.sequence(feeder.setSpeeds(+1), Commands.waitSeconds(0.5), feeder.stop()));

        // TODO determine angles for medium and far
        NamedCommands.registerCommand("ArmForNear", arm.goToAngle(Rotation2d.fromDegrees(15), 1));
        NamedCommands.registerCommand("ArmForMedium", arm.goToAngle(Rotation2d.fromDegrees(20), 1));
        NamedCommands.registerCommand("ArmForFar", arm.goToAngle(Rotation2d.fromDegrees(50), 1));
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

    private Command resetRobotPose() {
        return Commands.runOnce(
                        () -> {
                            drivetrain.setPose(
                                    new Pose2d(
                                            new Translation2d(
                                                    drivetrain.isRedAlliance() ? Constants.FIELD_WIDTH_METERS - 1.35 : 1.35,
                                                    5.55),
                                            new Rotation2d()));
                        },
                        drivetrain)
                .ignoringDisable(true);
    }
}
