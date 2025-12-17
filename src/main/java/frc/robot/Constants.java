package frc.robot;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
public class Constants {

    public static final class DriveConstants {
        public static final int  LEFT_MOTOR_1_ID = 21;
        public static final int  LEFT_MOTOR_2_ID = 20;
        public static final boolean LEFT_MOTORS_REVERSED = false;

        public static final int RIGHT_MOTOR_1_ID = 11;
        public static final int RIGHT_MOTOR_2_ID = 10;
        public static final boolean RIGHT_MOTORS_REVERSED = true;

        public static final double DRIVE_PID_P = 0.022;
        public static final double DRIVE_PID_I = 0;
        public static final double DRIVE_PID_D = 0.001;
    }

    public final class NotePhysicsConstants {
        public static final double DRAG_CONSTANT = 0.5;
        public static final double CROSS_SECTION_AREA = 0.01;
        public static final Mass MASS = Kilogram.of(0.145);
        public static final double FLUID_DENSITY = 1.2754;
        public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.80665);
    }

    public static final class ControlConstants {
        public static final double FORWARD_SPEED = 0.2;
        public static final double RIGHT_SPEED = 0.1;

        public static final double ARM_SPEED = 1;
    }

    public final class ArmConstants {
        public static final int ARM_MOTOR_ID = 4;
        public static final double ARM_PID_P = 0.009;//3;
        public static final double ARM_PID_I = 0;
        public static final double ARM_PID_D = 0;
        public static final boolean ARM_MOTOR_INVERTED = true;
        public static final boolean ARM_ENCODER_INVERTED = false;
        public static final double ARM_ZERO_OFFSET = 0.0672100;
        public static final double ARM_STATIONARY_CONSTANT = 0.069;
        public static final Angle ARM_UPPER_LIMIT = Degrees.of(80);
        public static final Angle ARM_LOWER_LIMIT = Degrees.of(-15);
        public static final Translation3d ARM_PIVOT_OFFSET = new Translation3d(
            0,
            0,
            0.18
        );
        public static final Distance ARM_PIVOT_NOTE_OFFSET = Meter.of(0.22);
        public static final Angle ARM_ALLOWED_ERROR = Radian.of(0.02);
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_ID_1 = 2;
        public static final int SHOOTER_MOTOR_ID_2 = 3;
        public static final double SHOOTER_P = 0.001;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;
        public static final double SHOOTER_DIVISOR = 4;
        public static final double SHOOTER_SLOPE = 0.000196;
        public static final double UPPER_SHOOTER_RATIO = 1.1;
        public static final AngularVelocity SHOOTER_ANGULAR_VELOCITY = RPM.of(4700);
        public static final AngularVelocity SHOOTER_MAX_ANGULAR_VELOCITY = RPM.of(5000);
        public static final AngularAcceleration  SHOOTER_MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(5); 
        public static final AngularVelocity SHOOTER_ALLOWED_ERROR = RPM.of(1);
    }

    public static final class TargetConstants {
        public static final LinearVelocity SHOOTER_VELOCITY = MetersPerSecond.of(9.1);
        public static final Translation3d TARGET_POSITION = new Translation3d(3.5, 1,1.2);
        public static final int MAX_STEPS = 30;
        public static final int TPS = 30;
    }

    public static final class LimelightConstants {
        public static final String LIMELIGHT_NAME = "limelight";
    }

    public static final class IndexerConstants{
        public static final int INDEX_MOTOR_ID = 0;
        public static final boolean INDEX_MOTOR_INVERSED = true;
        public static final int INDEX_PROXIMITY_PORT = 0;

    }
}
