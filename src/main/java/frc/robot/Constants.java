package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public final class DriveConstants {
        public static final int LEFT_MOTOR_1_ID = 10;
        public static final int LEFT_MOTOR_2_ID = 11;
        public static final boolean LEFT_MOTORS_REVERSED = false;

        public static final int RIGHT_MOTOR_1_ID = 20;
        public static final int RIGHT_MOTOR_2_ID = 21;
        public static final boolean RIGHT_MOTORS_REVERSED = true;

        public static final Distance WHEEL_BASE_WIDTH = Meters.of(1);

        public static final int GYRO_ID = 1;
    }

    public final class ControlConstants {
        public static double FOWARD_SPEED = 0.25;
        public static double RIGHT_SPEED = 0.25;
    }

    public final class ArmConstants {
        public static final double GRAVITY_COMPENSATION = 0;
        public static final int ARM_NEO_MOTOR_ID = 1;
        public static final double kP_CONSTANT = 0;
        public static final double kI_CONSTANT = 0;
        public static final double kD_CONSTANT = 0;
    }
    public final class IndexerConstants {
        public static final int INDEX_MOTOR_ID = 2;
        public static final boolean INDEX_MOTOR_INVERSED = true;
       
        public static final int INDEX_PROXIMITY_PORT = 1;
    }
}
