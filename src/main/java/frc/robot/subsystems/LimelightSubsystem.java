package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import java.util.Set;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    // 2026 FRC Hub tag IDs
   private static final Set<Integer> RED_TAGS  = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
   private static final Set<Integer> BLUE_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    // Teleop auto-align toggle
    private boolean autoAlignEnabled = false;

    // Reject vision poses built from tags farther than this - distant tags are noisy
    private static final double MAX_TRUSTED_TAG_DISTANCE_METERS = 6.0;

    private CommandSwerveDrivetrain drivetrain;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        setPipeline(0);
    }

    /** Called once from RobotContainer after the drivetrain is constructed. */
    public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    // -------------------------------------------------------------------------
    // Auto-align toggle (teleop)
    // -------------------------------------------------------------------------

    public void toggleAutoAlign() {
        autoAlignEnabled = !autoAlignEnabled;
    }

    public void setAutoAlign(boolean enabled) {
        autoAlignEnabled = enabled;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }

    // -------------------------------------------------------------------------
    // Alliance tag filtering
    // -------------------------------------------------------------------------

    /** Returns true if the currently visible tag belongs to our alliance's Hub. */
    public boolean isValidAllianceTag() {
        int id = getAprilTagID();
        if (id == -1) return false;

        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        return alliance.get() == Alliance.Red
                ? RED_TAGS.contains(id)
                : BLUE_TAGS.contains(id);
    }

    /** Returns true if we have a target AND it's a valid alliance Hub tag. */
    public boolean hasValidTarget() {
        return hasTarget() && isValidAllianceTag();
    }

    // -------------------------------------------------------------------------
    // Core targeting values
    // -------------------------------------------------------------------------

    /** Horizontal offset from crosshair to target (degrees). Negative = target is left. */
    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /** Vertical offset from crosshair to target (degrees). */
    public double getTY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    /** Target area as % of image (0-100). Useful for rough distance estimation. */
    public double getTA() {
        return table.getEntry("ta").getDouble(0.0);
    }

    /** Returns true if the Limelight has any valid target. */
    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /** Returns the ID of the primary AprilTag in view, or -1 if none. */
    public int getAprilTagID() {
        return (int) table.getEntry("tid").getDouble(-1.0);
    }

    // -------------------------------------------------------------------------
    // Camera mode & pipeline control
    // -------------------------------------------------------------------------

    /** 0 = vision processing, 1 = driver camera (no processing) */
    public void setDriverMode(boolean driverMode) {
        table.getEntry("camMode").setValue(driverMode ? 1 : 0);
    }

    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setValue(pipeline);
    }

    // -------------------------------------------------------------------------
    // LED control
    // -------------------------------------------------------------------------

    /** 0 = pipeline default, 1 = force off, 2 = force blink, 3 = force on */
    public void setLEDMode(int mode) {
        table.getEntry("ledMode").setValue(mode);
    }

    public void turnOnLEDs()  { setLEDMode(3); }
    public void turnOffLEDs() { setLEDMode(1); }

    // -------------------------------------------------------------------------
    // Vision pose fusion - corrects drivetrain odometry drift using AprilTags
    // -------------------------------------------------------------------------

    private void updateVisionPose() {
        if (drivetrain == null) return;

        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return;

        LimelightHelpers.PoseEstimate estimate = alliance.get() == Alliance.Red
            ? LimelightHelpers.getBotPoseEstimate_wpiRed("limelight")
            : LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (estimate == null || estimate.tagCount == 0) return;
        if (estimate.avgTagDist > MAX_TRUSTED_TAG_DISTANCE_METERS) return;

        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);

        SmartDashboard.putNumber("Vision Pose X",        estimate.pose.getTranslation().getX());
        SmartDashboard.putNumber("Vision Pose Y",        estimate.pose.getTranslation().getY());
        SmartDashboard.putNumber("Vision Tag Count",     estimate.tagCount);
        SmartDashboard.putNumber("Vision Avg Tag Dist",  estimate.avgTagDist);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight Has Target",    hasTarget());
        SmartDashboard.putBoolean("Limelight Valid Hub Tag", isValidAllianceTag());
        SmartDashboard.putBoolean("Auto Align Enabled",      autoAlignEnabled);
        SmartDashboard.putNumber("Limelight TX",             getTX());
        SmartDashboard.putNumber("Limelight TY",             getTY());
        SmartDashboard.putNumber("Limelight Tag ID",         getAprilTagID());
        SmartDashboard.putNumber("Limelight Distance (in)",
            hasTarget() ? LimelightHelpers.getTargetPose3d_CameraSpace("limelight")
                .getTranslation().getNorm() * 39.3701 : -1);

        updateVisionPose();
    }
}