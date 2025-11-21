package frc.robot;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
public class Constants {

    public static final class DriveConstants {
        public static final int LEFT_MOTOR_1_ID = 10;
        public static final int LEFT_MOTOR_2_ID = 11;
        public static final boolean LEFT_MOTORS_REVERSED = false;

        public static final int RIGHT_MOTOR_1_ID = 20;
        public static final int RIGHT_MOTOR_2_ID = 21;
        public static final boolean RIGHT_MOTORS_REVERSED = true;
        public static final Distance WHEEL_BASE_WIDTH = Meters.of(1);

        public static final int GYRO_ID = 1;
    }

    public final class NotePhysicsConstants {
        public static double DRAG_CONSTANT = 0.1;
        public static double CROSS_SECTION_AREA = 0.01;
        public static double MASS = 0.145;
        public static double FLUID_DENSITY = 1.2574;
    }

    public static final class ControlConstants {
        public static double FORWARD_SPEED = 0.25;
        public static double RIGHT_SPEED = 0.25;
    }

    public final class ArmContants {
        public static final int ARM_MOTOR_1 = 1;
        public static final int ARM_PID_P = 3;
        public static final int ARM_PID_I = 0;
        public static final int ARM_PID_D = 0;
        public static final int ARM_ZERO_OFFSET = 0;
        public static final float ARM_STATIONARY_CONSTANT = 1;
    }

    public static final class ShooterConstants {
        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double SHOOTER_MOTOR_ENCODER_OFFSET = 0;
        public static final double SPEED = 0;
        public static int SHOOTER_MOTOR = 1;
    }
}
