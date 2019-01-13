package com.team2502.robot2019;

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

    public static final long DEFAULT_ACTION_PERIOD = 20;
    public static final TimeUnit DEFAULT_ACTION_PERIOD_UNIT = TimeUnit.MILLISECONDS;

    /**
     * Variables that pertain to auto
     */
    public static class Autonomous
    {

        private Autonomous() { }
    }

    public static class Physical
    {
        private Physical() { }

        /**
         * Contains encoder conversion constants and details about the drivetrain wheels
         */
        public static class DriveTrain
        {
            //TODO: Measure wheels
            public static final double WHEEL_DIAMETER_INCH = UNDEFINED;
            public static final double WHEEL_DIAMETER_FT = WHEEL_DIAMETER_INCH / 12F;

            public static final double WHEEL_REV_TO_ENC_REV_LOW = 4.285F;
            public static final double WHEEL_REV_TO_ENC_REV_HIGH = 2.083F;

            //TODO: Remeasure
            public static final double MAX_FPS_SPEED = 18.0F;

            //TODO: Measure/Calculate
            public static final double MAX_FPS2_ACCEL = UNDEFINED;

            //TODO: Figure out if 2019 robot will even have 2 speeds
            public static final double SHIFT_UP_THRESHOLD = UNDEFINED;
            public static final double SHIFT_DOWN_THRESHOLD = UNDEFINED;

            /**
             * Threshold below which the joystick is considered to be at a 0 position
             */
            public static final double THRESHOLD = 1e-2;


            private DriveTrain() { }
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

            private Encoder() { }
        }
    }
}
