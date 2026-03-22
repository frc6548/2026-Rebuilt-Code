package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.commands.AlignToAprilTagCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShiftTracker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.commands.HoodCommand;
import frc.robot.subsystems.SpindexerSubsystem;
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

import edu.wpi.first.networktables.NetworkTableInstance;
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

  private final SpindexerSubsystem Spindexerkraken = new SpindexerSubsystem();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final ShooterSubsystem Shooterkraken = new ShooterSubsystem();
  private final HoodSubsystem HoodSubsystem = new HoodSubsystem(9);
  private final IntakeSubsystem Intakekraken = new IntakeSubsystem();
  private final IntakeLiftSubsystem intakeLiftSubsystem = new IntakeLiftSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final ShiftTracker shiftTracker = new ShiftTracker();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Register named commands BEFORE building auto chooser
NamedCommands.registerCommand("Intake down and wait",
    new SequentialCommandGroup(
        new InstantCommand(() -> {
            System.out.println(">>> Forcing intake to POSITION_DOWN: " + IntakeLiftSubsystem.POSITION_DOWN);
            intakeLiftSubsystem.setPosition(IntakeLiftSubsystem.POSITION_DOWN);
        }, intakeLiftSubsystem),
        new WaitCommand(1.0)
 ));
    NamedCommands.registerCommand("Intake deactive",
        new InstantCommand(() -> Intakekraken.stop(), Intakekraken));
    NamedCommands.registerCommand("Intake up and deactive",
        new ParallelCommandGroup(
            new IntakeLiftCommand(intakeLiftSubsystem, IntakeLiftSubsystem.POSITION_UP),
            new InstantCommand(() -> Intakekraken.stop(), Intakekraken)
        ));
   NamedCommands.registerCommand("Intake dump",
    new ParallelDeadlineGroup(
        new WaitCommand(2.0),
        new IntakeLiftCommand(intakeLiftSubsystem, IntakeLiftSubsystem.Dump),
        new IntakeCommand(Intakekraken)
    ));
    System.out.println(">>> Registered: Intake dump");

    NamedCommands.registerCommand("Shooter Sequence",
        new ShootCommand(Shooterkraken, Spindexerkraken));
    NamedCommands.registerCommand("Align To Hub",
        new AlignToAprilTagCommand(limelight, HoodSubsystem, true));
        System.out.println(">>> Registered: Align To Hub");
    NamedCommands.registerCommand("Wait 1 Second",
        new WaitCommand(1.0));
        NamedCommands.registerCommand("Intake active",
    new IntakeCommand(Intakekraken));

    // Build auto chooser AFTER AutoBuilder is configured by drivetrain
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                 .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                 .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
        )
    );

    intakeLiftSubsystem.setDefaultCommand(
        Commands.run(() -> intakeLiftSubsystem.stop(), intakeLiftSubsystem)
    );

    HoodSubsystem.setDefaultCommand(
        Commands.run(() -> HoodSubsystem.stop(), HoodSubsystem)
    );

    joystick.rightTrigger().onTrue(new ShootCommand(Shooterkraken, Spindexerkraken));
    joystick.y().onTrue(new HoodCommand(HoodSubsystem, frc.robot.subsystems.HoodSubsystem.Hardstop));
    joystick.a().onTrue(new HoodCommand(HoodSubsystem, frc.robot.subsystems.HoodSubsystem.TestPosition));
    joystick.x().onTrue(new IntakeCommand(Intakekraken));
    joystick.povUp().onTrue(new IntakeLiftCommand(intakeLiftSubsystem, IntakeLiftSubsystem.POSITION_UP));
    joystick.povDown().onTrue(new IntakeLiftCommand(intakeLiftSubsystem, IntakeLiftSubsystem.POSITION_DOWN));
   joystick.rightBumper().onTrue(new SequentialCommandGroup(
    new ParallelDeadlineGroup(
        new WaitCommand(2.0),
        new IntakeLiftCommand(intakeLiftSubsystem, IntakeLiftSubsystem.Dump),
        new IntakeCommand(Intakekraken)
    ),
    new ParallelCommandGroup(
        new IntakeLiftCommand(intakeLiftSubsystem, IntakeLiftSubsystem.POSITION_DOWN),
        new InstantCommand(() -> Intakekraken.stop(), Intakekraken)
    )
));
    joystick.povRight().onTrue(Commands.runOnce(() -> HoodSubsystem.NegativeincrementPosition(), HoodSubsystem));
    joystick.povLeft().onTrue(Commands.runOnce(() -> HoodSubsystem.PosotiveincrementPosition(), HoodSubsystem));

    // B button — toggle hood auto-adjust
    joystick.b().onTrue(Commands.runOnce(() -> limelight.toggleAutoAlign()));
    new Trigger(() -> limelight.isAutoAlignEnabled())
        .whileTrue(new AlignToAprilTagCommand(limelight, HoodSubsystem));

    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void SetDriveTrainSpeed(double speed) {
    MaxSpeed = speed;
    drive.withDeadband(MaxSpeed * 0.1);
  }
}