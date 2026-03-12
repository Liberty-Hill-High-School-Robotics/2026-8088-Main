package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.util.Optional;

// From https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
public class ActiveHubTracker {
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in
    // teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) { // 2:20 - 2:10
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 130 - Constants.kTimingOffset) { // 2:10 - ~2:08
      // Shoothing early for shift 1, hub is active.
      return true;
    } else if (matchTime > 105 + Constants.kTimingOffset) { // 2:10 - ~1:47
      // Shift 1, hub is active if shift 1 is active.
      return shift1Active;
    } else if (matchTime > 105 - Constants.kTimingOffset) { // ~1:47 - ~1:43
      // Shoothing late for shift 1 or early for shift 2, hub is active.
      return true;
    } else if (matchTime > 80 + Constants.kTimingOffset) { // ~1:45 - ~1:22
      // Shift 2, hub is active if shift 1 is not active.
      return !shift1Active;
    } else if (matchTime > 80 - Constants.kTimingOffset) { // ~1:22 - ~1:18
      // Shoothing late for shift 2 or early for shift 3, hub is active.
      return true;
    } else if (matchTime > 55 + Constants.kTimingOffset) { // ~1:18 - ~57
      // Shift 3, hub is active if shift 1 is active.
      return shift1Active;
    } else if (matchTime > 55 - Constants.kTimingOffset) { // ~57 - ~53
      // Shooting late for shift 3 or early for shift 4, hub is active.
      return true;
    } else if (matchTime > 30 + Constants.kTimingOffset) { // ~53 - ~32
      // Shift 4, hub is active if shift 1 is not active.
      return !shift1Active;
    } else if (matchTime > 30 - Constants.kTimingOffset) { // ~32 - ~28
      // Shooting late for shift 4 or early for endgame, hub is active.
      return true;
    } else { // 30 - 0
      // End game, hub always active.
      return true;
    }
  }
}
