package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

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
    private final double mass = Constants.NotePhysicsConstants.MASS;
    private final double fluidDensity = Constants.NotePhysicsConstants.FLUID_DENSITY;

    public ProjectileSubsystem() {

    }

    /**
     * TargetSolution
     * 
     * @param errorCode Stores if the solver has encountered an error. Anything over 0 is an error:
     *  <ul>
     *       <li>1 = Ideal pitch cannot reach</li>
     *       <li>2 = Yaw exceeds 360 degrees (2pi)</li>
     *       <li>3 = Pitch exceeds arm upper limit</li>
     *       <li>4 = Pitch exceeds arm lower limit</li>
     *       <li>5 = Height error exceeds 0.1 meters</li>
     *       <li>6 = Yaw error exceeds 0.1 radians</li>
     *  </ul>
     * @param launchPitch Stores the launch pitch of the projectile as an {@link Angle}
     * @param launchYaw Stores the launch yaw of the projectile as an {@link Angle}
     */
    public record TargetSolution(
        int errorCode,
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

        double discriminant = Math.pow(launchSpeedMPS, 4) - (Math.pow(9.8, 2) * Math.pow(horizontalDistance.in(Meter), 2)) + (2 * 9.8 * Math.pow(launchSpeedMPS, 2) * -heightOffset.in(Meter));

        if (discriminant < 0) {
            return null;
        }

        double square_root = Math.sqrt(discriminant);

        double numerator = (Math.pow(launchSpeedMPS, 2)) - square_root;
        double denominator = 9.8 * horizontalDistance.in(Meter);

        return Radians.of(Math.atan(numerator / denominator));
    }

    /**
     * Calculates the derivatives of the system state for runge kutta 4.
     * 
     * @param systemState A double array of length 6 that stoes the state of the system in the order:
     *  <ul>
     *       <li>X position</li>
     *       <li>Y position</li>
     *       <li>Z position</li>
     *       <li>X velocity</li>
     *       <li>Y velocity</li>
     *       <li>Z velocity</li>
     *  </ul>
     * @param dragConstant The drag constant of the projectile
     * @return A double list of length 6 that stores the derivatives of the system state in the order:
     *  <ul>
     *       <li>X velocity</li>
     *       <li>Y velocity</li>
     *       <li>Z velocity</li>
     *       <li>X acceleration</li>
     *       <li>Y acceleration</li>
     *       <li>Z acceleration</li>
     *  </ul>
     */
    private double[] rungeKuttaDerivative(double[] systemState, double dragConstant) {
        double totalVelocity = Math.sqrt(Math.pow(systemState[3], 2) + Math.pow(systemState[4], 2) + Math.pow(systemState[5], 2));

        double xDrag = 0;
        double yDrag = 0;
        double zDrag = 0;

        if (totalVelocity >= 1e-9) {
            xDrag = -dragConstant * totalVelocity * systemState[3];
            yDrag = -dragConstant * totalVelocity * systemState[4];
            zDrag = -dragConstant * totalVelocity * systemState[5];
        }

        double xAccel = (xDrag / mass);
        double yAccel = (yDrag / mass);
        double zAccel = (zDrag / mass) - 9.8;

        return new double[]{systemState[3], systemState[4], systemState[5], xAccel, yAccel, zAccel};
    }

    /**
     * Applies a derivative of a system state to the system state using a factor.
     * 
     * @param systemState1 A double array of length 6 that stores the inital system state
     * @param systemState2 A double array of length 6 that stores the derivative of the inital system state
     * @param multiplier The factor to use when applying the derivative
     * @param deltaTime The time between steps of the simulation
     * @return The intermediate system state
     */
    private double[] rungeKuttaIntermediate(double[] systemState1, double[] systemState2, double multiplier, Time deltaTime) {
        double[] finalSystemState = new double[6];
        
        for (int index = 0; index < 6; index++) {
            finalSystemState[index] = systemState1[index] + multiplier * systemState2[index] * deltaTime.in(Seconds);
        }

        return finalSystemState;
    }

    /**
     * Performs an integration step for runge kutta 4.
     * 
     * @param systemState A double array of length 6 that stores the state of the system in the order:
     *  <ul>
     *       <li>X position</li>
     *       <li>Y position</li>
     *       <li>Z position</li>
     *       <li>X velocity</li>
     *       <li>Y velocity</li>
     *       <li>Z velocity</li>
     *  </ul>
     * @param dragConstant The drag constant of the projectile
     * @param deltaTime The time between steps of the simulation
     * @return A double array of length 6 that stores the state of the system after this step in the order:
     *  <ul>
     *       <li>X position</li>
     *       <li>Y position</li>
     *       <li>Z position</li>
     *       <li>X velocity</li>
     *       <li>Y velocity</li>
     *       <li>Z velocity</li>
     *  </ul>
     */
    private double[] rungeKuttaStep(double[] systemState, double dragConstant, Time deltaTime) {
        double[] k1SystemState = rungeKuttaDerivative(systemState, dragConstant);
        double[] k2SystemState = rungeKuttaDerivative(rungeKuttaIntermediate(systemState, k1SystemState, 0.5, deltaTime), dragConstant);
        double[] k3SystemState = rungeKuttaDerivative(rungeKuttaIntermediate(systemState, k2SystemState, 0.5, deltaTime), dragConstant);
        double[] k4SystemState = rungeKuttaDerivative(rungeKuttaIntermediate(systemState, k3SystemState, 1, deltaTime), dragConstant);

        double[] finalSystemState = new double[6];

        for (int index = 0; index < 6; index++) {
            double average_slope = (k1SystemState[index] + (2 * k2SystemState[index]) + (2 * k3SystemState[index]) + k4SystemState[index]) / 6;

            finalSystemState[index] = systemState[index] + (average_slope * deltaTime.in(Seconds));
        }

        return finalSystemState;
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

        Translation3d position = Constants.ArmConstants.ARM_PIVOT_OFFSET;

        double noteVerticalOffset = Math.sin(launchPitch.in(Radians)) * Constants.ArmConstants.ARM_PIVOT_NOTE_OFFSET.in(Meter);
        double noteForwardOffset = Math.cos(launchPitch.in(Radians)) * Constants.ArmConstants.ARM_PIVOT_NOTE_OFFSET.in(Meter);

        double noteXOffset = Math.cos(launchYaw.in(Radians)) * noteForwardOffset;
        double noteYOffset = Math.sin(launchYaw.in(Radians)) * noteForwardOffset;

        position = position.plus(new Translation3d(
            noteXOffset,
            noteYOffset,
            noteVerticalOffset
        ));

        Translation3d velocity = new Translation3d(
            (launchSpeed.in(MetersPerSecond) * Math.cos(launchPitch.in(Radians)) * Math.cos(launchYaw.in(Radians))) + robotVelocity.getX(),
            (launchSpeed.in(MetersPerSecond) * Math.cos(launchPitch.in(Radians)) * Math.sin(launchYaw.in(Radians))) + robotVelocity.getY(),
            launchSpeed.in(MetersPerSecond) * Math.sin(launchPitch.in(Radians))
        );

        Time deltaTime = Seconds.of(1.0 / tps);

        double dragConstant = 0.5 * dragCoefficient * fluidDensity * crossSectionArea;

        double horizontalDistance = Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2));

        Translation3d[] path = new Translation3d[]{new Translation3d(), new Translation3d()};

        for (int step = 0; step < 5 * tps; step++) {
            double[] systemState = rungeKuttaStep(
                new double[]{
                    position.getX(),
                    position.getY(),
                    position.getZ(),
                    velocity.getX(),
                    velocity.getY(),
                    velocity.getZ()
                }, dragConstant, deltaTime);

            position = new Translation3d(
                systemState[0],
                systemState[1],
                systemState[2]
            );

            velocity = new Translation3d(
                systemState[3],
                systemState[4],
                systemState[5]
            );

            path[0] = path[1];

            path[1] = position;

            if (Math.sqrt(Math.pow(position.getX(), 2) + Math.pow(position.getY(), 2)) >= horizontalDistance) {
                break;
            }
        }

        return path;
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
            landing_yaw - targetDirectAngle.in(Radians)
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

        Angle launchAnglePitch1Temp = calculateLaunchPitchIdeal(launchSpeed, horizontalDistance, Meter.of(targetPosition.getZ()));

        if (launchAnglePitch1Temp == null) {
            return new TargetSolution(1, Radians.of(0.0), Radians.of(0.0));
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

        int solutionFound = 0;
        if (Math.abs(launchAngleYaw1) > (Math.PI * 2)) {
            solutionFound = 2;
        } else if (launchAnglePitch1 > Constants.ArmConstants.ARM_UPPER_LIMIT.in(Radians)) {
            solutionFound = 3;
        } else if (launchAnglePitch1 < Constants.ArmConstants.ARM_LOWER_LIMIT.in(Radians)) {
            solutionFound = 4;
        } else if (Math.abs(launchError1[0]) > 0.1) {
            solutionFound = 5;
        } else if (Math.abs(launchError1[1]) > 0.1) {
            solutionFound = 6;
        }

        return new TargetSolution(solutionFound, Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1));
    }
}