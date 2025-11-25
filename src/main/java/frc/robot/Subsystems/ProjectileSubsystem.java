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
    private final double dragCoefficent = Constants.NotePhysicsConstants.DRAG_CONSTANT;
    private final double crossSectionArea = Constants.NotePhysicsConstants.CROSS_SECTION_AREA;
    private final double mass = Constants.NotePhysicsConstants.MASS;
    private final double fluidDensity = Constants.NotePhysicsConstants.FLUID_DENSITY;

    public ProjectileSubsystem() {

    }

    public record TargetSolution(
        Boolean solutionFound,
        Angle launchPitch,
        Angle launchYaw
    ) {};

    private Angle calculateLaunchPitchIdeal(LinearVelocity launchSpeed, Distance horizontalDistance, Distance heightOffset) {
        double launchSpeedMPS = launchSpeed.in(MetersPerSecond);

        double square_root = Math.sqrt(Math.pow(launchSpeedMPS, 4) - (Math.pow(9.8, 2) * Math.pow(horizontalDistance.in(Meter), 2)) + (2 * 9.8 * Math.pow(launchSpeedMPS, 2) * -heightOffset.in(Meter)));

        double numerator = (Math.pow(launchSpeedMPS, 2)) - square_root;
        double denominator = 9.8 * horizontalDistance.in(Meter);

        return Radians.of(Math.atan(numerator / denominator));
    }

    private double[] rungeKuttaDerivitive(double[] systemState, double dragConstant) {
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

    private double[] rungeKuttaIntermediate(double[] systemState1, double[] systemState2, double multiplier, Time deltaTime) {
        double[] finalSystemState = new double[6];
        
        for (int index = 0; index < 6; index++) {
            finalSystemState[index] = systemState1[index] + multiplier * systemState2[index] * deltaTime.in(Seconds);
        }

        return finalSystemState;
    }

    private double[] rungeKuttaStep(double[] systemState, double dragConstant, Time deltaTime) {
        double[] k1SystemState = rungeKuttaDerivitive(systemState, dragConstant);
        double[] k2SystemState = rungeKuttaDerivitive(rungeKuttaIntermediate(systemState, k1SystemState, 0.5, deltaTime), dragConstant);
        double[] k3SystemState = rungeKuttaDerivitive(rungeKuttaIntermediate(systemState, k2SystemState, 0.5, deltaTime), dragConstant);
        double[] k4SystemState = rungeKuttaDerivitive(rungeKuttaIntermediate(systemState, k3SystemState, 1, deltaTime), dragConstant);

        double[] finalSystemState = new double[6];

        for (int index = 0; index < 6; index++) {
            double average_slope = (k1SystemState[index] + (2 * k2SystemState[index]) + (2 * k3SystemState[index]) + k4SystemState[index]) / 6;

            finalSystemState[index] = systemState[index] + (average_slope * deltaTime.in(Seconds));
        }

        return finalSystemState;
    }

    public Translation3d[] simulateLaunch(LinearVelocity launchSpeed, Angle launchPitch, Angle launchYaw, Translation2d robotVelocity, Translation3d targetPosition, int tps) {

        Translation3d position = new Translation3d();
        Translation3d velocity = new Translation3d(
            (launchSpeed.in(MetersPerSecond) * Math.cos(launchPitch.in(Radians)) * Math.cos(launchYaw.in(Radians))) + robotVelocity.getX(),
            (launchSpeed.in(MetersPerSecond) * Math.cos(launchPitch.in(Radians)) * Math.sin(launchYaw.in(Radians))) + robotVelocity.getY(),
            launchSpeed.in(MetersPerSecond) * Math.sin(launchPitch.in(Radians))
        );

        Time deltaTime = Seconds.of(1.0 / tps);

        double dragConstant = 0.5 * dragCoefficent * fluidDensity * crossSectionArea;

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

    private double[] calculateHeightError(LinearVelocity launchSpeed, Angle launchPitch, Angle launchYaw, Translation2d robotVelocity, Translation3d targetPosition, Angle targetDirectAngle, Distance horizontalDistance, int tps) {
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

    public TargetSolution calculateLaunchAngleSimulation(LinearVelocity launchSpeed, Translation2d robotVelocity, Translation3d targetPosition, int maxSteps, int tps) {
        Distance horizontalDistance = Meter.of(Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2)));

        double targetDirectAngle = Math.atan2(targetPosition.getY(), targetPosition.getX());

        double launchAnglePitch1 = calculateLaunchPitchIdeal(launchSpeed, horizontalDistance, Meter.of(targetPosition.getZ())).in(Radians);
        double launchAnglePitch2 = launchAnglePitch1 + 0.1;
        double launchAngleYaw1 = targetDirectAngle - 0.1;
        double launchAngleYaw2 = targetDirectAngle + 0.1;

        // Height Error, Yaw Error
        double[] launchError1 = calculateHeightError(launchSpeed, Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1), robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);
        double[] launchError2 = calculateHeightError(launchSpeed, Radians.of(launchAnglePitch2), Radians.of(launchAngleYaw2), robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);

        for (int steps = 0; steps < maxSteps; steps++) {
            if (Math.abs(launchError1[0]) < 1e-7 && Math.abs(launchError1[1]) < 1e-6) {
                break;
            }

            double weightPitch = 0;
            if (Math.abs(launchError1[0]) > 1e-9) {
                weightPitch = (launchAnglePitch1 - launchAnglePitch2) / (launchError1[0] - launchError2[0]);
            }

            double weightYaw = 0;
            if (Math.abs(launchError1[1]) > 1e-9) {
                weightYaw = (launchAngleYaw1 - launchAngleYaw2) / (launchError1[1] - launchError2[1]);
            }

            launchAnglePitch2 = launchAnglePitch1;
            launchAngleYaw2 = launchAngleYaw1;
            launchError2 = launchError1;

            launchAnglePitch1 -= (launchError1[0] * weightPitch);
            launchAngleYaw1 -= (launchError1[1] * weightYaw);

            // Height Error, Yaw Error
            launchError1 = calculateHeightError(launchSpeed, Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1), robotVelocity, targetPosition, Radians.of(targetDirectAngle), horizontalDistance, tps);
        }

        Boolean solutionFound = true;
        if (Math.abs(launchAngleYaw1) > (Math.PI * 2)) {
            solutionFound = false;
        } else if (launchAnglePitch1 > Constants.ArmContants.ARM_UPPER_LIMIT.in(Radians)) {
            solutionFound = false;
        } else if (launchAnglePitch1 < Constants.ArmContants.ARM_LOWER_LIMIT.in(Radians)) {
            solutionFound = false;
        }

        return new TargetSolution(solutionFound, Radians.of(launchAnglePitch1), Radians.of(launchAngleYaw1));
    }
}