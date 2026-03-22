package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToAprilTagCommand extends Command {

    // -------------------------------------------------------------------------
    // Hood stability constants
    // -------------------------------------------------------------------------
    private static final int kStableCount = 10; // loops hood must be stable

    // -------------------------------------------------------------------------
    // Hood distance-to-position lookup table
    // *** TUNE THESE VALUES ON THE ROBOT ***
    // -------------------------------------------------------------------------
    private static final double[] DISTANCES      = { 66,   72,   78,   83   }; // inches
    private static final double[] HOOD_POSITIONS = { 0.0,  0.31, 0.63, 0.89 }; // motor rotations

    private final LimelightSubsystem limelight;
    private final HoodSubsystem      hood;
    private final boolean            isAuto;

    private int    stableLoops = 0;
    private double hoodTarget  = 0.0;

    /**
     * Teleop constructor — hood auto-adjusts, runs until B toggled off.
     */
    public AlignToAprilTagCommand(LimelightSubsystem limelight, HoodSubsystem hood) {
        this.limelight = limelight;
        this.hood      = hood;
        this.isAuto    = false;
        addRequirements(hood);
    }

    /**
     * Auto constructor — hood auto-adjusts, finishes when hood is stable.
     * NO drivetrain control — PathPlanner handles all driving.
     */
    public AlignToAprilTagCommand(LimelightSubsystem limelight, HoodSubsystem hood,
            boolean isAuto) {
        this.limelight = limelight;
        this.hood      = hood;
        this.isAuto    = isAuto;
        addRequirements(hood);
    }

    // -------------------------------------------------------------------------
    // Distance calculation using LimelightHelpers 3D pose
    // -------------------------------------------------------------------------
    private double getDistanceInches() {
        try {
            return LimelightHelpers.getTargetPose3d_CameraSpace("limelight")
                    .getTranslation().getNorm() * 39.3701;
        } catch (Exception e) {
            return -1;
        }
    }

    // -------------------------------------------------------------------------
    // Linear interpolation of hood position from distance
    // -------------------------------------------------------------------------
    private double getHoodPositionForDistance(double distanceInches) {
        if (distanceInches <= DISTANCES[0]) return HOOD_POSITIONS[0];
        if (distanceInches >= DISTANCES[DISTANCES.length - 1]) return HOOD_POSITIONS[HOOD_POSITIONS.length - 1];
        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (distanceInches >= DISTANCES[i] && distanceInches <= DISTANCES[i + 1]) {
                double t = (distanceInches - DISTANCES[i]) / (DISTANCES[i + 1] - DISTANCES[i]);
                return HOOD_POSITIONS[i] + t * (HOOD_POSITIONS[i + 1] - HOOD_POSITIONS[i]);
            }
        }
        return HOOD_POSITIONS[0];
    }

    @Override
    public void initialize() {
        stableLoops = 0;
        System.out.println(">>> AlignToAprilTag STARTED (auto=" + isAuto + ")");
    }

    @Override
    public void execute() {
        if (!limelight.hasValidTarget()) {
            SmartDashboard.putNumber("Limelight Distance (in)", -1);
            SmartDashboard.putNumber("Hood Target Position", -1);
            return;
        }

        double distance = getDistanceInches();
        hoodTarget = getHoodPositionForDistance(distance);
        SmartDashboard.putNumber("Limelight Distance (in)", distance);
        SmartDashboard.putNumber("Hood Target Position", hoodTarget);
        hood.setPosition(hoodTarget);

        if (isAuto && hood.atTarget(hoodTarget)) {
            stableLoops++;
        } else {
            stableLoops = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(">>> AlignToAprilTag STOPPED" + (interrupted ? " (interrupted)" : " (finished)"));
    }

    @Override
    public boolean isFinished() {
        if (!isAuto) return false;
        return stableLoops >= kStableCount;
    }
}