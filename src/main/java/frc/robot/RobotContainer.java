package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.commands.HoodCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeLiftSubsystem;
import frc.robot.commands.IntakeLiftCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SpindexerSubsystem Spindexerkraken = new SpindexerSubsystem();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final ShooterSubsystem Shooterkraken = new ShooterSubsystem();
  private final HoodSubsystem HoodSubsystem = new HoodSubsystem(9);
  private final IntakeSubsystem Intakekraken = new IntakeSubsystem();
  private final IntakeLiftSubsystem intakeLiftSubsystem = new IntakeLiftSubsystem();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  // private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);


  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
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
 // HoodSubsystem.setDefaultCommand(
   // Commands.run(() -> HoodSubsystem.stop(), HoodSubsystem)
 // );

    joystick.a().onTrue(new SpindexerCommand(Spindexerkraken));
    joystick.y().onTrue(new ShooterCommand(Shooterkraken));
    joystick.x().onTrue(new HoodCommand(HoodSubsystem, frc.robot.subsystems.HoodSubsystem.Hardstop));
    joystick.b().onTrue(new HoodCommand(HoodSubsystem, frc.robot.subsystems.HoodSubsystem.TestPosition));
    joystick.rightTrigger().onTrue(new IntakeCommand(Intakekraken));
    joystick.leftBumper().debounce(0.1).onTrue(new IntakeLiftCommand(intakeLiftSubsystem, IntakeLiftSubsystem.POSITION_UP));
    joystick.rightBumper().debounce(0.1).onTrue(new IntakeLiftCommand(intakeLiftSubsystem, IntakeLiftSubsystem.POSITION_DOWN));
  
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; // Replace with your actual autonomous command
  }

  public void SetDriveTrainSpeed(double speed){
    MaxSpeed = speed;
    drive.withDeadband(MaxSpeed * 0.1); // Update the deadband based on the new MaxSpeed
  }
}