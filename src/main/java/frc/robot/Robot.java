package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // Set to false to disable vision entirely
  private final boolean kUseLimelight = true;

  // True until autonomous starts — used to seed pose during disabled
  private boolean beforeMatch = true;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    CommandScheduler.getInstance().run();

    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // Must set robot orientation BEFORE getting pose estimate
      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);

      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

      // Strict filters to prevent yaw drift and bad measurements:
      // - Require 2+ tags (single tag yaw is unreliable)
      // - Reject tags further than 4 meters
      // - Reject when spinning fast (blurry/delayed vision during rotation)
      // NOTE: addVisionMeasurement only updates field position tracking,
      //       it NEVER touches motors or overrides driver input
      if (llMeasurement != null
          && llMeasurement.tagCount >= 2
          && llMeasurement.avgTagDist < 4.0
          && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            llMeasurement.pose,
            llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {
    System.out.println(">>> DISABLED - cancelling all commands");
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    // Before match starts, use MegaTag1 to seed the robot pose
    // MegaTag1 is more reliable when stationary
    if (kUseLimelight && beforeMatch) {
      PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            llMeasurement.pose,
            llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println(">>> Auto command: " + m_autonomousCommand);
    if (m_autonomousCommand != null) {
      System.out.println(">>> Scheduling auto command");
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    } else {
      System.out.println(">>> AUTO COMMAND IS NULL - check autoChooser in RobotContainer!");
    }
    beforeMatch = false;
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}