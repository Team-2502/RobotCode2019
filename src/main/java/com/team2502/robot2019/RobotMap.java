package com.team2502.robot2019;

import java.util.HashMap;
import java.util.Map;

/**
 * Exclusively for defining Button ID's, Solenoid ID's, Motor ID's, and other kinds of ID's
 * Stuff like conversion constants DO NOT belong here
 */
public class RobotMap
{

    private RobotMap() { }

    /**
     * Define Joystick ID's and button ID's
     */
    public static final class Joystick
    {
        public static final int JOYSTICK_DRIVE_LEFT = 1;
        public static final int JOYSTICK_DRIVE_RIGHT = 0;
        public static final int JOYSTICK_FUNCTION = 2;

        private Joystick() { }

        /**
         * Define Button ID's, which should be used in OI.java
         */
        public static final class Button
        {
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


        private Motor() { }
    }

    /**
     * Define Solenoid ID's
     */
    public static final class Solenoid
    {
        private Solenoid() { }
    }
}