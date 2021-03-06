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
        public static final int JOYSTICK_SIDE_PANEL = 3;

        private Joystick()
        {
        }

        /**
         * Define Button ID's, which should be used in OI.java
         */
        public static final class Button
        {
            public static final int BUTTON_HATCH_PUSHER = 1;
            public static final int BUTTON_KILL_ACTIONS = 14;
            public static final int BUTTON_ENABLE_AUTO_ALIGN = 1;

            public static final int BUTTON_RUN_CARGO_ACTIVE_FWD_BOTTOM = 5;
            public static final int BUTTON_RUN_CARGO_ACTIVE_BWD_BOTTOM = 3;

            public static final int BUTTON_RUN_CARGO_ACTIVE_FWD_TOP = 6;
            public static final int BUTTON_RUN_CARGO_ACTIVE_BWD_TOP = 4;

            public static final int BUTTON_CLIMBER_STRUCTURE_UP = 11;
            public static final int BUTTON_CLIMBER_STRUCTURE_DOWN = 12;

            public static final int BUTTON_LEFT_CLIMB_UP = 7;
            public static final int BUTTON_LEFT_CLIMB_DOWN = 9;
            public static final int BUTTON_RIGHT_CLIMB_UP = 8;
            public static final int BUTTON_RIGHT_CLIMB_DOWN = 10;

            public static final int BUTTON_SWITCH_CAMERA = 2;
            public static final int BUTTON_CRAWL = 1;
            public static final int BUTTON_TOGGLE_FLIP_OUT = 7;
            public static final int BUTTON_TOGGLE_OBA = 8;
            public static final int BUTTON_DRIVER_ANTI_TIP = 2;

            public static final int BUTTON_HUD_L0 = 1;
            public static final int BUTTON_HUD_L1 = 2;
            public static final int BUTTON_HUD_L2 = 3;
            public static final int BUTTON_HUD_R0 = 4;
            public static final int BUTTON_HUD_R1 = 5;
            public static final int BUTTON_HUD_R2 = 6;

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
        public static final int CARGO_UPPER_BELT = 6;

        public static final int CLIMBER_LEFT = 7;
        public static final int CLIMBER_RIGHT = 8;
        public static final int CLIMBER_CLAW_LEFT = 9;
        public static final int CLIMBER_CLAW_RIGHT = 10;

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
        public static final int FLIP_OUTS = 3;
        public static final int OBA = 2;

        private Solenoid() { }
    }
}