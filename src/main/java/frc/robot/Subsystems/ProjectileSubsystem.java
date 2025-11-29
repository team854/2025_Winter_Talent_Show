package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem responsible for computing ballistic solutions for a projectile
 * (note, amp, trap, etc.) given robot pose, target pose, and launch speed.
 *
 * This version fixes:
 * - Yaw computation using atan2 (correct quadrants).
 * - Time-of-flight math via closed-form launch-angle solution.
 * - Angle limit checks.
 * - Unit handling: all physics done in SI doubles, units used at the edges.
 */
public class ProjectileSubsystem extends SubsystemBase {

    // Maximum allowed absolute yaw in radians
    private static final double MAX_YAW_ABS_RAD = Math.toRadians(150.0);

    /**
     * Status codes for the {@link TargetSolution}.
     *
     * 0 = VALID
     * 1 = NO_SOLUTION (no ballistic solution at this speed / geometry)
     * 2 = YAW_OUT_OF_RANGE
     * 3 = PITCH_TOO_HIGH (above arm upper limit)
     * 4 = PITCH_TOO_LOW (below arm lower limit)
     * 5/6 reserved for future error checks (horizontal/vertical miss).
     */
    public static final class Status {
        public static final int VALID = 0;
        public static final int NO_SOLUTION = 1;
        public static final int YAW_OUT_OF_RANGE = 2;
        public static final int PITCH_TOO_HIGH = 3;
        public static final int PITCH_TOO_LOW = 4;
        public static final int HORIZ_ERROR_TOO_LARGE = 5;
        public static final int VERT_ERROR_TOO_LARGE = 6;

        private Status() {
        }
    }

    /**
     * Result of the ballistic solver: status code + desired launch angles.
     * Angles are given in WPILib {@link Angle} units (radians).
     */
    public static class TargetSolution {
        public final int solutionFound;
        public final Angle pitch;
        public final Angle yaw;

        public TargetSolution(int solutionFound, Angle pitch, Angle yaw) {
            this.solutionFound = solutionFound;
            this.pitch = pitch;
            this.yaw = yaw;
        }
    }

    public ProjectileSubsystem() {
        // Add any logging / initialization here if needed.
    }

    /**
     * Compute the required launch pitch and yaw to hit a target with a projectile with
     * given speed, assuming simple ballistic motion under gravity (no drag).
     *
     * @param robotPose   Robot pose in the field coordinate frame.
     * @param targetPose  Target pose (e.g., center of speaker opening) in the same frame.
     * @param launchSpeed Launch speed of the projectile.
     * @return TargetSolution containing pitch/yaw and a status code.
     */
    public TargetSolution solveForTarget(Pose3d robotPose, Pose3d targetPose, LinearVelocity launchSpeed) {

        // --- Convert to a relative vector from shooter origin to target in field frame ---

        Translation3d robotPosition = robotPose.getTranslation();
        Translation3d targetPosition = targetPose.getTranslation();

        double robotX = robotPosition.getX();
        double robotY = robotPosition.getY();
        double robotZ = robotPosition.getZ();

        double targetX = targetPosition.getX();
        double targetY = targetPosition.getY();
        double targetZ = targetPosition.getZ();

        // Relative position from robot to target (in meters)
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double dz = targetZ - robotZ;

        // Horizontal distance in the X-Y plane
        double horizontalDistance = Math.hypot(dx, dy);

        // Guard against degenerate geometry
        if (horizontalDistance <= 1e-6) {
            // Target is "on top" of unc bot horizontally. not solvable with this model
            return new TargetSolution(Status.NO_SOLUTION, Radians.of(0.0), Radians.of(0.0));
        }

        // --- Convert units to raw doubles for physics calculations ideally---

        double v = launchSpeed.in(MetersPerSecond);                  // m/s
        double g = Constants.Projectile.GRAVITY; // m/s^2 (positive magnitude)

        // Additional safety check
        if (v <= 0.0) {
            return new TargetSolution(Status.NO_SOLUTION, Radians.of(0.0), Radians.of(0.0));
        }

        // --- Compute yaw using atan2. Makes sure quadrant is correct ---

        double yaw = Math.atan2(dy, dx);

        if (Math.abs(yaw) > MAX_YAW_ABS_RAD) {
            // Yaw will cause the shooter to slam into the ground or frame (big no no)
            return new TargetSolution(Status.YAW_OUT_OF_RANGE, Radians.of(0.0), Radians.of(yaw));
        }

        // ---------- Relevant info: ---------- \\
        //
        // --- Solve for pitch using closed-form ballistic formula with different heights ---
        //
        // Equation for launch angle θ (relative to horizontal) for a projectile:
        //   R = horizontal distance, h = vertical offset (dz), g = gravity, v = launch speed
        //
        //   tan(θ) = (v^2 ± sqrt(v^4 - g (g R^2 + 2 h v^2))) / (g R)
        //
        // Compute both roots and pick the one that is within the arm limits (+ optionally low arc).

        double R = horizontalDistance;
        double h = dz;

        double v2 = v * v;
        double v4 = v2 * v2;

        double termUnderSqrt = v4 - g * (g * R * R + 2.0 * h * v2);

        if (termUnderSqrt < 0.0) {
            // No real solution (impossible to hit desired target)
            return new TargetSolution(Status.NO_SOLUTION, Radians.of(0.0), Radians.of(yaw));
        }

        double sqrtTerm = Math.sqrt(termUnderSqrt);

        // Two possible tan(theta) values (high arc, low arc)
        double tanTheta1 = (v2 + sqrtTerm) / (g * R);
        double tanTheta2 = (v2 - sqrtTerm) / (g * R);

        double theta1 = Math.atan(tanTheta1);
        double theta2 = Math.atan(tanTheta2);

        // Arm limits (in radians)
        double minPitchRad = Constants.ArmConstants.ARM_LOWER_LIMIT.in(Radians);
        double maxPitchRad = Constants.ArmConstants.ARM_UPPER_LIMIT.in(Radians);

        // Pick a valid pitch within arm limits, prefer lower magnitude (low arc) if both valid.
        Double chosenPitchRad = null;

        if (theta1 >= minPitchRad && theta1 <= maxPitchRad) {
            chosenPitchRad = theta1;
        }
        if (theta2 >= minPitchRad && theta2 <= maxPitchRad) {
            if (chosenPitchRad == null || Math.abs(theta2) < Math.abs(chosenPitchRad)) {
                chosenPitchRad = theta2;
            }
        }

        if (chosenPitchRad == null) {
            // Both candidate pitches are out of range. Decide whether "too high" or "too low".
            // If both attempts are above max, call it too high; if both below min, too low.
            boolean bothHigh = theta1 > maxPitchRad && theta2 > maxPitchRad;
            boolean bothLow = theta1 < minPitchRad && theta2 < minPitchRad;

            if (bothHigh) {
                return new TargetSolution(Status.PITCH_TOO_HIGH,
                        Radians.of(maxPitchRad), Radians.of(yaw));
            } else if (bothLow) {
                return new TargetSolution(Status.PITCH_TOO_LOW,
                        Radians.of(minPitchRad), Radians.of(yaw));
            } else {
                // Mixed but neither within limits so treat as no solution.
                return new TargetSolution(Status.NO_SOLUTION, Radians.of(0.0), Radians.of(yaw));
            }
        }

        double pitchRad = chosenPitchRad;

        // --- Compute time of flight from horizontal motion: R = v cos(θ) t ---

        double cosPitch = Math.cos(pitchRad);
        if (Math.abs(cosPitch) < 1e-6) {
            // Vertical shot not handled by this solver
            return new TargetSolution(Status.NO_SOLUTION, Radians.of(pitchRad), Radians.of(yaw));
        }

        double tFlight = R / (v * cosPitch);

        if (tFlight <= 0.0) {
            return new TargetSolution(Status.NO_SOLUTION, Radians.of(pitchRad), Radians.of(yaw));
        }

        // ChatGPT suggested this
        //
        // --- (Optional) forward check: where do we actually land with this pitch/yaw/time?
        //
        // NOTE: With the same simplified model, this will match exactly (up to floating point).
        // Any "error" here would only come from numeric precision, so error thresholds are not
        // very meaningful unless we introduce a more realistic model (e.g., drag).
        //
        // Keeping the structure in case you want to expand, but we do NOT reject on these.

        // Horizontal displacement using chosen pitch & yaw
        double vx = v * cosPitch * Math.cos(yaw);
        double vy = v * cosPitch * Math.sin(yaw);
        double vz = v * Math.sin(pitchRad);

        double predX = robotX + vx * tFlight;
        double predY = robotY + vy * tFlight;
        double predZ = robotZ + vz * tFlight - 0.5 * g * tFlight * tFlight;

        double horizError = Math.hypot(predX - targetX, predY - targetY);
        double vertError = predZ - targetZ;

        // These could be re-enabled once you have a more complex model:
        // int status = Status.VALID;
        // if (Math.abs(horizError) > POSITION_TOLERANCE_METERS) {
        //     status = Status.HORIZ_ERROR_TOO_LARGE;
        // } else if (Math.abs(vertError) > POSITION_TOLERANCE_METERS) {
        //     status = Status.VERT_ERROR_TOO_LARGE;
        // }

        int status = Status.VALID;

        return new TargetSolution(status, Radians.of(pitchRad), Radians.of(yaw));
    }

    /**
     * Convenience wrapper that takes distances rather than full poses, if you ever
     * want a simpler API 
     * probably good to use with limelight values
     */
    public TargetSolution solveForTargetRelative(Distance x, Distance y, Distance z, LinearVelocity launchSpeed) {
        Pose3d robotPose = new Pose3d(0.0, 0.0, 0.0, robotRotationNoOp());
        Pose3d targetPose = new Pose3d(
                x.in(Meter),
                y.in(Meter),
                z.in(Meter),
                robotRotationNoOp());

        return solveForTarget(robotPose, targetPose, launchSpeed);
    }

    /**
     * Helper: neutral rotation (no-op) for simple relative pose construction.
     */
    private edu.wpi.first.math.geometry.Rotation3d robotRotationNoOp() {
        return new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, 0.0);
    }
}
