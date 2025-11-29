package frc.robot;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
public class Constants {

    public static final class DriveConstants {
        public static final int  LEFT_MOTOR_1_ID = 21;
        public static final boolean LEFT_MOTORS_REVERSED = false;

        public static final int RIGHT_MOTOR_1_ID = 11;
        public static final boolean RIGHT_MOTORS_REVERSED = true;
    }

    public final class NotePhysicsConstants {
        public static final double DRAG_CONSTANT = 0.1;
        public static final double CROSS_SECTION_AREA = 0.01;
        public static final double MASS = 0.145;
        public static final double FLUID_DENSITY = 1.2574;
    }

    public static final class ControlConstants {
        public static final double FORWARD_SPEED = 0.25;
        public static final double RIGHT_SPEED = 0.5;

        public static final double ARM_SPEED = 1;
    }

    public final class ArmConstants {
        public static final int ARM_MOTOR_ID = 4;
        public static final double ARM_PID_P = 0.01;//3;
        public static final double ARM_PID_I = 0;
        public static final double ARM_PID_D = 0;
        public static final boolean ARM_MOTOR_INVERTED = true;
        public static final boolean ARM_ENCODER_INVERTED = true;
        public static final double ARM_ZERO_OFFSET = 0.1023485;
        public static final double ARM_STATIONARY_CONSTANT = 0.047;
        public static final Angle ARM_UPPER_LIMIT = Degrees.of(90);
        public static final Angle ARM_LOWER_LIMIT = Degrees.of(-10);
        public static final Translation3d ARM_PIVOT_OFFSET = new Translation3d(
            0,
            0,
            0
        );
        public static final Distance ARM_PIVOT_NOTE_OFFSET = Meter.of(0.1);
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_ID_1 = 2;
        public static final int SHOOTER_MOTOR_ID_2 = 3;
        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final AngularVelocity SHOOTER_ANGULAR_VELOCITY = RPM.of(500);
        public static final AngularVelocity SHOOTER_MAX_ANGULAR_VELOCITY = RPM.of(5000);
        public static final AngularAcceleration  SHOOTER_MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(5); 
        public static final AngularVelocity SHOOTER_ALLOWED_ERROR = RPM.of(1);
    }

    public static final class LimelightConstants {
        public static final String LIMELIGHT_NAME = "limelight";
    }

    public final class Projectile {
        public static final double GRAVITY = 9.80665;
    }
}
