package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToAprilTagCommand extends Command {

    // -------------------------------------------------------------------------
    // Tuning constants — adjust these on the robot
    // -------------------------------------------------------------------------
    private static final double kP           = 0.04; // Rotational P gain
    private static final double kMinOutput   = 0.05; // Minimum power to overcome static friction
    private static final double kMaxOutput   = 0.4;  // Cap rotation speed (rad/s)
    private static final double kTolerance   = 1.0;  // Degrees of TX considered "aligned"
    private static final int    kStableCount = 10;   // Loops within tolerance before finishing

    private final LimelightSubsystem      limelight;
    private final CommandSwerveDrivetrain drive;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private int stableLoops = 0;

    public AlignToAprilTagCommand(LimelightSubsystem limelight, CommandSwerveDrivetrain drive) {
        this.limelight = limelight;
        this.drive     = drive;
        addRequirements(limelight, drive);
    }

    @Override
    public void initialize() {
        stableLoops = 0;
    }

    @Override
    public void execute() {
        if (!limelight.hasValidTarget()) {
            drive.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double tx = limelight.getTX();

        double rotation = tx * kP;

        // Apply minimum output to overcome friction
        if (Math.abs(rotation) < kMinOutput && Math.abs(tx) > kTolerance) {
            rotation = Math.copySign(kMinOutput, rotation);
        }

        // Clamp to max output
        rotation = Math.max(-kMaxOutput, Math.min(kMaxOutput, rotation));

        drive.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotation));

        if (Math.abs(tx) < kTolerance) {
            stableLoops++;
        } else {
            stableLoops = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return stableLoops >= kStableCount;
    }
}