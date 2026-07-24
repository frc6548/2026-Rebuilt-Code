package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToAprilTagCommand extends Command {

    // Loops hood must be on target before finishing in auto
    private static final int STABLE_COUNT = 10;

    // -------------------------------------------------------------------------
    // Hood distance-to-position lookup table
    // *** TUNE THESE VALUES ON THE ROBOT ***
    // Format: distance (inches) → hood position (motor rotations)
    // -------------------------------------------------------------------------
    private static final double[] DISTANCES      = { 66.0, 67.8, 82.0, 83.0, 88.0 };
    private static final double[] HOOD_POSITIONS = { 0.42333984375, 0.42333984375, 0.501953125, 0.501953125, 0.524853125 };

    private final LimelightSubsystem limelight;
    private final HoodSubsystem      hood;
    private final boolean            isAuto;

    private int    stableLoops = 0;
    private double hoodTarget  = 0.0;

    /** Teleop constructor - runs until B is toggled off. */
    public AlignToAprilTagCommand(LimelightSubsystem limelight, HoodSubsystem hood) {
        this(limelight, hood, false);
    }

    /** Auto constructor - finishes when hood is stable on target. */
    public AlignToAprilTagCommand(LimelightSubsystem limelight, HoodSubsystem hood, boolean isAuto) {
        this.limelight = limelight;
        this.hood      = hood;
        this.isAuto    = isAuto;
        addRequirements(hood);
    }

    // Fallback hood position used when no tag is visible in auto
    private static final double FALLBACK_HOOD = 0.46; // midpoint of known range, tune as needed

    @Override
    public void initialize() {
        stableLoops = 0;
        System.out.println(">>> AlignToAprilTag STARTED (auto=" + isAuto + ")");
    }

    @Override
    public void execute() {
        if (!limelight.hasValidTarget()) {
            // No valid tag - use fallback position in auto, hold current in teleop
            if (isAuto) hood.setPosition(FALLBACK_HOOD);
            else hood.holdPosition();
            SmartDashboard.putNumber("Hood Auto Distance (in)", -1);
            SmartDashboard.putNumber("Hood Auto Target",        isAuto ? FALLBACK_HOOD : -1);
            stableLoops = 0;
            return;
        }

        double distance = getDistanceInches();
        hoodTarget = getHoodPositionForDistance(distance);

        hood.setPosition(hoodTarget);

        SmartDashboard.putNumber("Hood Auto Distance (in)", distance);
        SmartDashboard.putNumber("Hood Auto Target",        hoodTarget);

        if (isAuto && hood.atTarget(hoodTarget)) {
            stableLoops++;
        } else {
            stableLoops = 0;
        }
    }

    @Override
    public boolean isFinished() {
        if (!isAuto) return false;
        return stableLoops >= STABLE_COUNT;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(">>> AlignToAprilTag STOPPED" + (interrupted ? " (interrupted)" : " (finished)"));
        hood.snapshotPosition(); // hold wherever the hood landed
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private double getDistanceInches() {
        try {
            return LimelightHelpers.getTargetPose3d_CameraSpace("limelight")
                    .getTranslation().getNorm() * 39.3701;
        } catch (Exception e) {
            return -1;
        }
    }

    private double getHoodPositionForDistance(double distanceInches) {
        if (distanceInches <= DISTANCES[0])
            return HOOD_POSITIONS[0];
        if (distanceInches >= DISTANCES[DISTANCES.length - 1])
            return HOOD_POSITIONS[HOOD_POSITIONS.length - 1];

        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (distanceInches >= DISTANCES[i] && distanceInches <= DISTANCES[i + 1]) {
                double t = (distanceInches - DISTANCES[i]) / (DISTANCES[i + 1] - DISTANCES[i]);
                return HOOD_POSITIONS[i] + t * (HOOD_POSITIONS[i + 1] - HOOD_POSITIONS[i]);
            }
        }
        return HOOD_POSITIONS[0];
    }
}