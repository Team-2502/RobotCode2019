package com.team2502.robot2019;

import com.github.ezauton.core.pathplanning.purepursuit.LookaheadBounds;
import com.github.ezauton.core.robot.TankRobotConstants;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;

import java.util.concurrent.TimeUnit;

/**
 * Constant variables generally pertaining to Pure Pursuit and encoders
 * <br>
 * Notes
 * <ul>
 * <li>E (EVEL, ENC_RES, EPOS) is special encoder units</li>
 * <li>The placement of the encoder means that its</li>
 * </ul>
 */
public class Constants
{
    /**
     * Value is marked as deprecated to trigger a compile time warning to notify
     * end user that the code may not work unless they properly set the proper id's.
     */
    @Deprecated
    public static final int UNDEFINED = -1;

    public static final int INIT_TIMEOUT = 10;
    public static final int LOOP_TIMEOUT = 0;

    public static final int PER100MS_TO_SECONDS = 10;

    public static final long DEFAULT_ACTION_PERIOD = 20;
    public static final TimeUnit DEFAULT_ACTION_PERIOD_UNIT = TimeUnit.MILLISECONDS;

    /**
     * Variables that pertain to auto
     */
    public static class Autonomous
    {
        public static final String COPROCESSOR_MDNS_ADDR = "raspberrypi.local";
        public static final int PORT = 5800;

        public static LookaheadBounds getLookaheadBounds(DriveTrain dt) {
            return new LookaheadBounds(1, 8, 3, 10, dt.getVelocityEstimator());
        }
        private Autonomous() { }
    }

    public static class Physical
    {
        private Physical() { }

        /**
         * Contains encoder conversion constants and details about the drivetrain wheels
         */
        public static class DriveTrain implements TankRobotConstants
        {
            public static final double WHEEL_DIAMETER_INCH = 6.0F;
            public static final double WHEEL_DIAMETER_FT = WHEEL_DIAMETER_INCH / 12F;

            public static final double WHEEL_CIRCUMFERENCE_INCH = WHEEL_DIAMETER_INCH * Math.PI;
            public static final double WHEEL_CIRCUMFERENCE_FT = WHEEL_CIRCUMFERENCE_INCH / 12F;

            public static final double WHEEL_REV_TO_ENC_REV = 1.0F;

            public static final double ENC_UNITS_PER_ROT = 4096;

            /**
             * Multiply by enc units to get feet
             */
            public static final double ENC_UNITS_TO_FEET = (WHEEL_DIAMETER_FT  * Math.PI)/ (ENC_UNITS_PER_ROT);

            /**
             * Multiply by enc units/100 ms to get feet/sec
             */
            public static final double ENC_UNITS_TO_FPS = 10 * ENC_UNITS_TO_FEET;



            //TODO: Remeasure
            public static final double MAX_FPS_SPEED = 18.0F;

            //TODO: Measure/Calculate
            public static final double MAX_FPS2_ACCEL = 30;

            /**
             * Threshold below which the joystick is considered to be at a 0 position
             */
            public static final double THRESHOLD = 1e-2;

            public static final double LATERAL_WHEEL_DIST_FT = 25.5D / 12;
            public static final double ROBOT_LENGTH_FT = 2D;

            public static final DriveTrain TANK_ROBOT_CONSTANTS = new DriveTrain();
            public static final double DEFAULT_KF_LEFT = .6;
            public static final double DEFAULT_KF_RIGHT = 0.495;
            public static final int DEFAULT_KD = 0;
            public static final int DEFAULT_KI = 0;
            public static final double DEFAULT_KP = 0;

            private DriveTrain() { }

            @Override
            public double getLateralWheelDistance()
            {
                return LATERAL_WHEEL_DIST_FT;
            }
        }

        public static class CargoActive
        {
            public static final double SPEED_FWD = -1.0D;
            public static final double SPEED_BWD = 1.0D;
        }

        public static class Crawler
        {
            public static final double SPEED_FWD = 1.0D;
            public static final double SPEED_BWD = -1.0D;
        }

        public static class Climber
        {
            public static final double SPEED_UP = 1.0D;
            public static final double SPEED_DOWN = -1.0D;
        }

        public static class Encoder
        {
            /**
             * Represents the amount of encoder ticks in 1 encoder revolution
             */
            public static final double ENC_RES = 4096.0F;

            /**
             * Multiply this by "encoder speed units" to get RPM
             */
            public static final double EVEL_TO_RPM = (600.0F / ENC_RES);

            public static final double RAW_UNIT_PER_ROT = ENC_RES;
            public static final double RAW_UNIT_PER_FT = ENC_RES / DriveTrain.WHEEL_CIRCUMFERENCE_FT;

            public Encoder() { }
        }
    }


}
