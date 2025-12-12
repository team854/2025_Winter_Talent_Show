package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ProjectileSubsystem extends SubsystemBase {
    private final double dragCoefficient = Constants.NotePhysicsConstants.DRAG_CONSTANT;
    private final double crossSectionArea = Constants.NotePhysicsConstants.CROSS_SECTION_AREA;
    private final double mass = Constants.NotePhysicsConstants.MASS.in(Kilogram);
    private final double fluidDensity = Constants.NotePhysicsConstants.FLUID_DENSITY;
    private final double gravity = Constants.NotePhysicsConstants.GRAVITY.in(MetersPerSecondPerSecond);

    public ProjectileSubsystem() {

    }

    /**
     * TargetErrorCode
     * <ul>
     *       <li>NONE = No error</li>
     *       <li>IDEAL_PITCH = Ideal pitch cannot reach</li>
     *       <li>EXCESSIVE_YAW = Yaw exceeds 360 degrees (2pi)</li>
     *       <li>PITCH_UPPER_LIMIT = Pitch exceeds arm upper limit</li>
     *       <li>PITCH_LOWER_LIMIT = Pitch exceeds arm lower limit</li>
     *       <li>HEIGHT_ERROR_HIGH = Height error exceeds 0.1 meters</li>
     *       <li>YAW_ERROR_HIGH = Yaw error exceeds 0.1 radians</li>
     *  </ul>
     */
    public enum TargetErrorCode {
        NONE,
        IDEAL_PITCH,
        EXCESSIVE_YAW,
        PITCH_UPPER_LIMIT,
        PITCH_LOWER_LIMIT,
        HEIGHT_ERROR_HIGH,
        YAW_ERROR_HIGH
    }

    /**
     * TargetSolution
     * 
     * @param errorCode Stores if the solver has encountered an error
     * @param launchPitch Stores the launch pitch of the projectile as an {@link Angle}
     * @param launchYaw Stores the launch yaw of the projectile as an {@link Angle}
     */
    public record TargetSolution(
        TargetErrorCode errorCode,
        Angle launchPitch,
        Angle launchYaw
    ) {};

    /**
     * Calculates the launch pitch needed to reach a target not accounting for drag.
     * 
     * @param launchSpeed The launch speed of the projectile as a {@link LinearVelocity}
     * @param horizontalDistance The targets horizontal distance from the robot as a {@link Distance}
     * @param heightOffset The height offset of the target from the rrobot as a {@link Distance}
     * @return The {@link Angle} that the projectile should be launched. Null if the projectile cannot reach the target
     */
    private Angle calculateLaunchPitchIdeal(LinearVelocity launchSpeed, Distance horizontalDistance, Distance heightOffset) {
        double launchSpeedMPS = launchSpeed.in(MetersPerSecond);

        if (Math.abs(horizontalDistance.in(Meter)) < 1e-5) {
            return null; 
        }

        double discriminant = Math.pow(launchSpeedMPS, 4) - (Math.pow(gravity, 2) * Math.pow(horizontalDistance.in(Meter), 2)) + (2 * gravity * Math.pow(launchSpeedMPS, 2) * -heightOffset.in(Meter));

        if (discriminant < 0) {
            return null;
        }

        double square_root = Math.sqrt(discriminant);

        double numerator = (Math.pow(launchSpeedMPS, 2)) - square_root;
        double denominator = gravity * horizontalDistance.in(Meter);

        return Radians.of(Math.atan(numerator / denominator));
    }

    /**
     * Simuulates the launch of a projectile using runge kutta 4.
     * 
     * @param launchSpeed The launch velocity of the projectile as a {@link LinearVelocity}
     * @param launchPitch The launch pitch of the projectiile as a {@link Angle}
     * @param launchYaw The launch yaw of the projectile as a {@link Angle}
     * @param robotVelocity The velocity of the robot as a {@link Translation3d} in Meters/Second
     * @param targetPosition The target posititon in field relative cordinates centered at the robot in {@link Translation3d} in Meter
     * @param tps The ticks per second of the simulation
     * @return A {@link Translation3d} array of the last two positions of the projectile
     */
    public Translation3d[] simulateLaunch(LinearVelocity launchSpeed, Angle launchPitch, Angle launchYaw, Translation2d robotVelocity, Translation3d targetPosition, int tps) {

        double noteVerticalOffset = Math.sin(launchPitch.in(Radians)) * Constants.ArmConstants.ARM_PIVOT_NOTE_OFFSET.in(Meter);
        double noteForwardOffset = Math.cos(launchPitch.in(Radians)) * Constants.ArmConstants.ARM_PIVOT_NOTE_OFFSET.in(Meter);

        double noteXOffset = Math.cos(launchYaw.in(Radians)) * noteForwardOffset;
        double noteYOffset = Math.sin(launchYaw.in(Radians)) * noteForwardOffset;

        double posX = Constants.ArmConstants.ARM_PIVOT_OFFSET.getX() + noteXOffset;
        double posY = Constants.ArmConstants.ARM_PIVOT_OFFSET.getY() + noteYOffset;
        double posZ = Constants.ArmConstants.ARM_PIVOT_OFFSET.getZ() + noteVerticalOffset;

        double velX = (launchSpeed.in(MetersPerSecond) * Math.cos(launchPitch.in(Radians)) * Math.cos(launchYaw.in(Radians))) + robotVelocity.getX();
        double velY = (launchSpeed.in(MetersPerSecond) * Math.cos(launchPitch.in(Radians)) * Math.sin(launchYaw.in(Radians))) + robotVelocity.getY();
        double velZ = launchSpeed.in(MetersPerSecond) * Math.sin(launchPitch.in(Radians));

        double deltaTime = 1.0 / tps;

        double dragConstant = 0.5 * dragCoefficient * fluidDensity * crossSectionArea;

        double horizontalDistance = Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2));

        double prevX = posX;
        double prevY = posY;
        double prevZ = posZ;

        double k1vx, k1vy, k1vz, k1ax, k1ay, k1az;
        double k2vx, k2vy, k2vz, k2ax, k2ay, k2az;
        double k3vx, k3vy, k3vz, k3ax, k3ay, k3az;
        double k4vx, k4vy, k4vz, k4ax, k4ay, k4az;

        double magVel, dragFactor;

        for (int step = 0; step < 5 * tps; step++) {
            
            prevX = posX;
            prevY = posY;
            prevZ = posZ;

            // K1
            k1vx = velX;
            k1vy = velY;
            k1vz = velZ;
            magVel = Math.sqrt(Math.pow(k1vx, 2) + Math.pow(k1vy, 2) + Math.pow(k1vz, 2));
            dragFactor = -dragConstant * magVel;

            k1ax = ((k1vx * dragFactor) / mass);
            k1ay = ((k1vy * dragFactor) / mass);
            k1az = ((k1vz * dragFactor) / mass) - gravity;
            
            // K2
            k2vx = velX + (0.5 * k1ax * deltaTime);
            k2vy = velY + (0.5 * k1ay * deltaTime);
            k2vz = velZ + (0.5 * k1az * deltaTime);
            magVel = Math.sqrt(Math.pow(k2vx, 2) + Math.pow(k2vy, 2) + Math.pow(k2vz, 2));
            dragFactor = -dragConstant * magVel;

            k2ax = ((k2vx * dragFactor) / mass);
            k2ay = ((k2vy * dragFactor) / mass);
            k2az = ((k2vz * dragFactor) / mass) - gravity;

            // K3
            k3vx = velX + (0.5 * k2ax * deltaTime);
            k3vy = velY + (0.5 * k2ay * deltaTime);
            k3vz = velZ + (0.5 * k2az * deltaTime);
            magVel = Math.sqrt(Math.pow(k3vx, 2) + Math.pow(k3vy, 2) + Math.pow(k3vz, 2));
            dragFactor = -dragConstant * magVel;

            k3ax = ((k3vx * dragFactor) / mass);
            k3ay = ((k3vy * dragFactor) / mass);
            k3az = ((k3vz * dragFactor) / mass) - gravity;

            // K4
            k4vx = velX + (k3ax * deltaTime);
            k4vy = velY + (k3ay * deltaTime);
            k4vz = velZ + (k3az * deltaTime);
            magVel = Math.sqrt(Math.pow(k4vx, 2) + Math.pow(k4vy, 2) + Math.pow(k4vz, 2));
            dragFactor = -dragConstant * magVel;

            k4ax = ((k4vx * dragFactor) / mass);
            k4ay = ((k4vy * dragFactor) / mass);
            k4az = ((k4vz * dragFactor) / mass) - gravity;

            posX += (k1vx + 2 * k2vx + 2 * k3vx + k4vx) / 6.0 * deltaTime;
            posY += (k1vy + 2 * k2vy + 2 * k3vy + k4vy) / 6.0 * deltaTime;
            posZ += (k1vz + 2 * k2vz + 2 * k3vz + k4vz) / 6.0 * deltaTime;

            velX += (k1ax + 2 * k2ax + 2 * k3ax + k4ax) / 6.0 * deltaTime;
            velY += (k1ay + 2 * k2ay + 2 * k3ay + k4ay) / 6.0 * deltaTime;
            velZ += (k1az + 2 * k2az + 2 * k3az + k4az) / 6.0 * deltaTime;


            if (Math.sqrt(Math.pow(posX, 2) + Math.pow(posY, 2)) >= horizontalDistance) {
                break;
            }
        }

        return new Translation3d[]{
            new Translation3d(prevX, prevY, prevZ),
            new Translation3d(posX, posY , posZ)
        };
    }

    /**
     * Interpolates the position of the projectile between two points
     * 
     * @param path A double array that contains the last two positions of the projectile as a {@link Translation3d} in Meters
     * @param horizontalDistance The horizontal distance of the target as a {@link Distance}
     * @return The interpolated positon of the projectile as a {@link Translation3d} in Meters
     */
    private Translation3d interpolatePosition(Translation3d[] path, Distance horizontalDistance) {
        double horizontalDistance1 = Math.sqrt(Math.pow(path[0].getX(), 2) + Math.pow(path[0].getY(), 2));
        double horizontalDistance2 = Math.sqrt(Math.pow(path[1].getX(), 2) + Math.pow(path[1].getY(), 2));

        if (Math.abs(horizontalDistance2 - horizontalDistance1) < 1e-9) {
            return path[0];
        }

        double weight = (horizontalDistance.in(Meter) - horizontalDistance1) / (horizontalDistance2 - horizontalDistance1);

        return new Translation3d(
            path[0].getX() + ((path[1].getX() - path[0].getX()) * weight),
            path[0].getY() + ((path[1].getY() - path[0].getY()) * weight),
            path[0].getZ() + ((path[1].getZ() - path[0].getZ()) * weight)
        );
    }

    /**
     * Calculates the error of the projectiles landing point
     * 
     * @param launchSpeed The launch speed of the projectile as a {@link LinearVelocity}
     * @param launchPitch The launch pitch of the projectile as a {@link Angle}
     * @param launchYaw The launch yaw of the projectile as a {@link Angle}
     * @param robotVelocity The field relative velocity of the robot as a {@link LinearVelocity} in Meters/Second
     * @param targetPosition The field relative position of the target centered at the robot
     * @param targetDirectAngle The direct yaw {@link Angle} towards the target
     * @param horizontalDistance The {@link Distance} from the robot to the target 
     * @param tps The ticks per second of the simulation
     * @return The error in the simulation in the order:
     *  <ul>
     *       <li>Height error</li>
     *       <li>Yaw error</li>
     *  </ul>
     */
    private double[] calculateLaunchError(LinearVelocity launchSpeed, Angle launchPitch, Angle launchYaw, Translation2d robotVelocity, Translation3d targetPosition, Angle targetDirectAngle, Distance horizontalDistance, int tps) {
        Translation3d[] path = simulateLaunch(
            launchSpeed,
            launchPitch,
            launchYaw,
            robotVelocity,
            targetPosition,
            tps
        );

        Translation3d crossOverPoint = interpolatePosition(path, horizontalDistance);

        double landing_yaw = Math.atan2(crossOverPoint.getY(), crossOverPoint.getX());

        return new double[]{
            crossOverPoint.getZ() - targetPosition.getZ(),
            MathUtil.angleModulus(landing_yaw - targetDirectAngle.in(Radians))
        };
    }

    /**
     * 
     * @param launchSpeed The launch speed of the projectile as a {@link LinearVelocity}
     * @param robotVelocity The field relative velocity of the robot as a {@link Translation2d} in Meters/Second
     * @param targetPosition The robt relative position of the target (Rotation is field relative) as a {@link Translation3d} in Meters
     * @param maxSteps The max ammount of optimization steps (It can exit early if the error gets below a threshold)
     * @param tps The ticks per second that physics will be calculated at
     * @return The target solution
     */
    public TargetSolution calculateLaunchAngleSimulation(LinearVelocity launchSpeed, Translation2d robotVelocity, Translation3d targetPosition, int maxSteps, int tps) {
        Distance horizontalDistance = Meter.of(Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2)));

        double targetDirectAngle = Math.atan2(targetPosition.getY(), targetPosition.getX());

        Angle launchAnglePitch1Temp = calculateLaunchPitchIdeal(launchSpeed, horizontalDistance, Meter.of(targetPosition.getZ() - Constants.ArmConstants.ARM_PIVOT_OFFSET.getZ()));

        if (launchAnglePitch1Temp == null) {
            return new TargetSolution(TargetErrorCode.IDEAL_PITCH, Radians.of(0.0), Radians.of(0.0));
        }
        double launchAnglePitch1 = launchAnglePitch1Temp.in(Radians);
        double launchAnglePitch2 = launchAnglePitch1 + 0.1;
        double launchAngleYaw1 = targetDirectAngle - 0.1;
        double launchAngleYaw2 = targetDirectAngle + 0.1;

        // Height Error, Yaw Error
        double[] launchError1 = calculateLaunchError(launchSpeed, Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1), robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);
        double[] launchError2 = calculateLaunchError(launchSpeed, Radians.of(launchAnglePitch2), Radians.of(launchAngleYaw2), robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        for (int steps = 0; steps < maxSteps; steps++) {
            if (Math.abs(launchError1[0]) < 1e-7 && Math.abs(launchError1[1]) < 1e-6) {
                break;
            }

            double weightPitch = 0;
            if (Math.abs(launchError1[0]) > 1e-9 && Math.abs(launchError1[0] - launchError2[0]) > 1e-9) {
                weightPitch = (launchAnglePitch1 - launchAnglePitch2) / (launchError1[0] - launchError2[0]);
            }

            double weightYaw = 0;
            if (Math.abs(launchError1[1]) > 1e-9 && Math.abs(launchError1[1] - launchError2[1]) > 1e-9) {
                weightYaw = (launchAngleYaw1 - launchAngleYaw2) / (launchError1[1] - launchError2[1]);
            }

            launchAnglePitch2 = launchAnglePitch1;
            launchAngleYaw2 = launchAngleYaw1;
            launchError2 = launchError1;

            launchAnglePitch1 -= (launchError1[0] * weightPitch);
            launchAngleYaw1 -= (launchError1[1] * weightYaw);

            // Height Error, Yaw Error
            launchError1 = calculateLaunchError(launchSpeed, Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1), robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);
        }

        TargetErrorCode solutionFound = TargetErrorCode.NONE;
        if (Math.abs(launchAngleYaw1) > (Math.PI * 2)) {
            solutionFound = TargetErrorCode.EXCESSIVE_YAW;
        } else if (launchAnglePitch1 > Constants.ArmConstants.ARM_UPPER_LIMIT.in(Radians)) {
            solutionFound = TargetErrorCode.PITCH_UPPER_LIMIT;
        } else if (launchAnglePitch1 < Constants.ArmConstants.ARM_LOWER_LIMIT.in(Radians)) {
            solutionFound = TargetErrorCode.PITCH_LOWER_LIMIT;
        } else if (Math.abs(launchError1[0]) > 0.1) {
            solutionFound = TargetErrorCode.HEIGHT_ERROR_HIGH;
        } else if (Math.abs(launchError1[1]) > 0.1) {
            solutionFound = TargetErrorCode.YAW_ERROR_HIGH;
        }

        return new TargetSolution(solutionFound, Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1));
    }
}