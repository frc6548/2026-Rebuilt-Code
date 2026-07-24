package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.commands.AlignToAprilTagCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.commands.HoodCommand;
import frc.robot.subsystems.RollerFlowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeLiftSubsystem;
import frc.robot.commands.IntakeLiftCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Subsystems
    private final ShooterSubsystem        shooter    = new ShooterSubsystem();
    private final RollerFlowSubsystem     rollerFlow = new RollerFlowSubsystem();
    private final IntakeSubsystem         intake     = new IntakeSubsystem();
    private final IntakeLiftSubsystem     intakeLift = new IntakeLiftSubsystem();
    private final HoodSubsystem           hood       = new HoodSubsystem(9);
    private final LimelightSubsystem      limelight  = new LimelightSubsystem();
    public  final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Drive
    private double MaxSpeed       = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry             logger   = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    // -------------------------------------------------------------------------
    // Named Commands (PathPlanner auto)
    // -------------------------------------------------------------------------
    private void registerNamedCommands() {

        NamedCommands.registerCommand("Intake down and wait",
            new SequentialCommandGroup(
                new InstantCommand(() -> intakeLift.setPosition(IntakeLiftSubsystem.POSITION_DOWN), intakeLift),
                new WaitCommand(0)
            ));

        NamedCommands.registerCommand("IntakeOn",
            new InstantCommand(() -> intake.spin(), intake));

        NamedCommands.registerCommand("IntakeOff",
            new InstantCommand(() -> intake.stop(), intake));

        NamedCommands.registerCommand("Intake up and deactive",
            new ParallelCommandGroup(
                new IntakeLiftCommand(intakeLift, IntakeLiftSubsystem.POSITION_UP),
                new InstantCommand(() -> intake.stop(), intake)
            ));

        NamedCommands.registerCommand("Intake dump",
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(2.0),
                    new IntakeLiftCommand(intakeLift, IntakeLiftSubsystem.DUMP),
                    new InstantCommand(() -> intake.spin(), intake)
                ),
                new InstantCommand(() -> intake.stop(), intake)
            ));

        NamedCommands.registerCommand("Shooter Sequence",
            new ShootCommand(shooter, rollerFlow));

        NamedCommands.registerCommand("Shooter Sequence with End",
            new ShootCommand(shooter, rollerFlow).withTimeout(7.0));

        NamedCommands.registerCommand("Align To Hub",
            new AlignToAprilTagCommand(limelight, hood, true).withTimeout(2.0));

        NamedCommands.registerCommand("Wait 1 Second",
            new WaitCommand(1.0));
    }

    // -------------------------------------------------------------------------
    // Bindings
    // -------------------------------------------------------------------------
    private void configureBindings() {

        // Default commands
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Hold lift position when no command is running
        intakeLift.setDefaultCommand(
            Commands.run(() -> intakeLift.holdPosition(), intakeLift)
        );

        // Actively hold hood position when no command is running
        hood.setDefaultCommand(
            Commands.run(() -> hood.holdPosition(), hood)
        );

        // Shoot sequence
        joystick.rightTrigger().onTrue(new ShootCommand(shooter, rollerFlow));

        // Hood UP (Y) and DOWN (A)
        joystick.y().onTrue(new HoodCommand(hood, HoodSubsystem.MAX));
        joystick.a().onTrue(new HoodCommand(hood, HoodSubsystem.HARDSTOP));

        // Intake toggle on X - press once to spin, press again to stop
        joystick.x().onTrue(new InstantCommand(() -> {
            if (intake.isRunning()) intake.stop();
            else intake.spin();
        }, intake));

        // Intake lift positions
        joystick.povUp().onTrue(new IntakeLiftCommand(intakeLift, IntakeLiftSubsystem.POSITION_UP));
        joystick.povDown().onTrue(new IntakeLiftCommand(intakeLift, IntakeLiftSubsystem.POSITION_DOWN));

        // Dump sequence
        joystick.rightBumper().onTrue(
            Commands.sequence(
                // Move to dump and hold there for 2 seconds while intake spins
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(() -> intakeLift.setPosition(IntakeLiftSubsystem.DUMP), intakeLift),
                    Commands.runOnce(() -> intake.spin(), intake)
                ),
                // Stop intake when done
                Commands.runOnce(() -> intake.stop(), intake)
            )
        );

        // Slow drivetrain to 50% when intake is running in teleop only
        double fullSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        new Trigger(() -> intake.isRunning() && !edu.wpi.first.wpilibj.DriverStation.isAutonomous())
            .onTrue(Commands.runOnce(() -> setDriveTrainSpeed(fullSpeed * 0.5)))
            .onFalse(Commands.runOnce(() -> setDriveTrainSpeed(fullSpeed)));

        // Hood increment - hold POV Left to go down, POV Right to go up by 0.1 per loop
        joystick.povLeft().whileTrue(Commands.run(() -> hood.incrementPosition(-0.1), hood));
        joystick.povLeft().onFalse(Commands.runOnce(() -> hood.snapshotPosition(), hood));
        joystick.povRight().whileTrue(Commands.run(() -> hood.incrementPosition(0.1), hood));
        joystick.povRight().onFalse(Commands.runOnce(() -> hood.snapshotPosition(), hood));
        joystick.b().onTrue(Commands.runOnce(() -> limelight.toggleAutoAlign()));
        new Trigger(() -> limelight.isAutoAlignEnabled())
            .whileTrue(new AlignToAprilTagCommand(limelight, hood));

        // Reset field-centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setDriveTrainSpeed(double speed) {
        MaxSpeed = speed;
        drive = drive.withDeadband(MaxSpeed * 0.1);
    }

    public void resetToDefaults() {
        setDriveTrainSpeed(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        intake.stop();
        rollerFlow.stop();
        shooter.stop();
        hood.resetToDefault();
        intakeLift.resetToDefault();
        limelight.setAutoAlign(false);
    }
}