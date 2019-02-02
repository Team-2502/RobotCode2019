package com.team2502.robot2019;

/**
 * Exclusively for defining Button ID's, Solenoid ID's, Motor ID's, and other kinds of ID's
 * Stuff like conversion constants DO NOT belong here
 */
public class RobotMap
{

    private RobotMap()
    {

    }

    /**
     * Define Joystick ID's and button ID's
     */
    public static final class Joystick
    {
        public static final int JOYSTICK_DRIVE_LEFT = 1;
        public static final int JOYSTICK_DRIVE_RIGHT = 0;
        public static final int JOYSTICK_FUNCTION = 2;

        private Joystick()
        {
        }

        /**
         * Define Button ID's, which should be used in OI.java
         */
        public static final class Button
        {
            public static final int BUTTON_HASH_PUSHER = 1;
            public static final int BUTTON_SWITCH_DIRECTION = 1;
            public static final int BUTTON_ABORT_AUTO = 11;

            public static final int RUN_CARGO_ACTIVE_BOTTOM = 3;
            public static final int RUN_CARGO_ACTIVE_BKWDS_BOTTOM = 5;

            public static final int RUN_CARGO_ACTIVE_TOP = 4;
            public static final int RUN_CARGO_ACTIVE_BKWDS_TOP = 6;
            private Button() { }
        }
    }

    /**
     * Define Motor Controller ID's
     */
    public static final class Motor
    {
        public static final int DRIVE_TRAIN_FRONT_RIGHT = 1;
        public static final int DRIVE_TRAIN_BACK_RIGHT = 2;
        public static final int DRIVE_TRAIN_FRONT_LEFT = 3;
        public static final int DRIVE_TRAIN_BACK_LEFT = 4;

        public static final int CARGO_LOWER_BELT = 5;
        public static final int CARGO_UPPER_BELT = 8;

        private Motor() { }
    }

    /**
     * Define Solenoid ID's
     */
    public class Solenoid
    {
        /*
         * HATCH_INTAKE ------ 4
         */

        public static final int HATCH_INTAKE = 4;

        private Solenoid() { }
    }
}