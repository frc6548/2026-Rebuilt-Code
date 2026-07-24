package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tracks the current Hub shift state for the 2026 FRC game REBUILT.
 *
 * Shift timing (match time counts down from 140s in teleop):
 *   > 130s : Transition shift - both hubs active
 *   > 105s : Shift 1 (25s)
 *   > 80s  : Shift 2 (25s)
 *   > 55s  : Shift 3 (25s)
 *   > 30s  : Shift 4 (25s)
 *   <= 30s : End game - both hubs active
 *
 * Game data: 'R' = red goes inactive first (active in shifts 2 & 4)
 *            'B' = blue goes inactive first (active in shifts 2 & 4)
 */
public class ShiftTracker extends SubsystemBase {

    public enum ShiftState {
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        END_GAME,
        UNKNOWN
    }

    @Override
    public void periodic() {
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        ShiftState shift = getCurrentShift(matchTime);
        boolean ourHubActive = isOurHubActive(shift, gameData);
        double timeLeftInShift = getTimeLeftInShift(matchTime, shift);

        // Current shift name
        SmartDashboard.putString("Current Shift", getShiftName(shift));

        // Time left in current shift
        SmartDashboard.putNumber("Time Left in Shift (s)", Math.max(0, timeLeftInShift));

        // Is our hub active?
        SmartDashboard.putBoolean("Our Hub Active", ourHubActive);

        // Which alliance's hub is active (R, B, Both, Unknown)
        SmartDashboard.putString("Active Hub", getActiveHubString(shift, gameData));

        // Raw match time for reference
        SmartDashboard.putNumber("Match Time (s)", matchTime);
    }

    // -------------------------------------------------------------------------
    // Determine current shift from match time
    // -------------------------------------------------------------------------
    private ShiftState getCurrentShift(double matchTime) {
        if (DriverStation.isAutonomousEnabled()) return ShiftState.AUTO;
        if (!DriverStation.isTeleopEnabled())   return ShiftState.UNKNOWN;

        if (matchTime > 130) return ShiftState.TRANSITION;
        if (matchTime > 105) return ShiftState.SHIFT_1;
        if (matchTime > 80)  return ShiftState.SHIFT_2;
        if (matchTime > 55)  return ShiftState.SHIFT_3;
        if (matchTime > 30)  return ShiftState.SHIFT_4;
        return ShiftState.END_GAME;
    }

    // -------------------------------------------------------------------------
    // Time remaining in the current shift
    // -------------------------------------------------------------------------
    private double getTimeLeftInShift(double matchTime, ShiftState shift) {
        return switch (shift) {
            case TRANSITION -> matchTime - 130;
            case SHIFT_1    -> matchTime - 105;
            case SHIFT_2    -> matchTime - 80;
            case SHIFT_3    -> matchTime - 55;
            case SHIFT_4    -> matchTime - 30;
            case END_GAME   -> matchTime;
            default         -> matchTime;
        };
    }

    // -------------------------------------------------------------------------
    // Is our alliance's hub currently active?
    // -------------------------------------------------------------------------
    private boolean isOurHubActive(ShiftState shift, String gameData) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        // Both hubs always active in these states
        if (shift == ShiftState.AUTO ||
            shift == ShiftState.TRANSITION ||
            shift == ShiftState.END_GAME ||
            shift == ShiftState.UNKNOWN) return true;

        if (gameData.isEmpty()) return true; // No data yet, assume active

        boolean redInactiveFirst = gameData.charAt(0) == 'R';

        // Shift 1 active alliance: opposite of who goes inactive first
        boolean shift1ActiveForRed  = !redInactiveFirst;
        boolean shift1ActiveForBlue = redInactiveFirst;

        boolean shift1Active = (alliance.get() == Alliance.Red)
                ? shift1ActiveForRed
                : shift1ActiveForBlue;

        return switch (shift) {
            case SHIFT_1 -> shift1Active;
            case SHIFT_2 -> !shift1Active;
            case SHIFT_3 -> shift1Active;
            case SHIFT_4 -> !shift1Active;
            default      -> true;
        };
    }

    // -------------------------------------------------------------------------
    // Which hub is active as a display string
    // -------------------------------------------------------------------------
    private String getActiveHubString(ShiftState shift, String gameData) {
        if (shift == ShiftState.AUTO ||
            shift == ShiftState.TRANSITION ||
            shift == ShiftState.END_GAME) return "Both";

        if (gameData.isEmpty()) return "Unknown";

        boolean redInactiveFirst = gameData.charAt(0) == 'R';

        boolean shift1ActiveForRed = !redInactiveFirst;

        boolean redActive = switch (shift) {
            case SHIFT_1 -> shift1ActiveForRed;
            case SHIFT_2 -> !shift1ActiveForRed;
            case SHIFT_3 -> shift1ActiveForRed;
            case SHIFT_4 -> !shift1ActiveForRed;
            default      -> true;
        };

        return redActive ? "Red" : "Blue";
    }

    // -------------------------------------------------------------------------
    // Human readable shift name
    // -------------------------------------------------------------------------
    private String getShiftName(ShiftState shift) {
        return switch (shift) {
            case AUTO       -> "Auto";
            case TRANSITION -> "Transition";
            case SHIFT_1    -> "Shift 1";
            case SHIFT_2    -> "Shift 2";
            case SHIFT_3    -> "Shift 3";
            case SHIFT_4    -> "Shift 4";
            case END_GAME   -> "End Game";
            default         -> "Unknown";
        };
    }
}