package com.team2502.robot2019;

import com.team2502.robot2019.command.teleop.CargoActiveCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The Operator Interface class
 */
public final class OI
{
    /**
     * Represents the left drive joystick
     *
     * @see OI
     */
    public static final Joystick JOYSTICK_DRIVE_LEFT = new Joystick(RobotMap.Joystick.JOYSTICK_DRIVE_LEFT);

    /**
     * Represents the right drive joystick
     *
     * @see OI
     */
    public static final Joystick JOYSTICK_DRIVE_RIGHT = new Joystick(RobotMap.Joystick.JOYSTICK_DRIVE_RIGHT);

    /**
     * Represents the function joystick
     *
     * @see OI
     */
    public static final Joystick JOYSTICK_FUNCTION = new Joystick(RobotMap.Joystick.JOYSTICK_FUNCTION);


    // Start defining buttons to be using
    // Names are self explanatory
    // Convention: Button variable names here should be the same as ID names in RobotMap

    public static final Button RUN_CARGO_ACTIVE_TOP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.RUN_CARGO_ACTIVE_TOP);
    public static final Button RUN_CARGO_ACTIVE_BKWDS_TOP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.RUN_CARGO_ACTIVE_BKWDS_TOP);

    public static final Button RUN_CARGO_ACTIVE_BOTTOM = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.RUN_CARGO_ACTIVE_BOTTOM);
    public static final Button RUN_CARGO_ACTIVE_BKWDS_BOTTOM = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.RUN_CARGO_ACTIVE_BKWDS_BOTTOM);
    /*
     * Runs when the first static method (usually OI#init()) is called
     * Called the "static initialization constructor"
     */
    static
    {
        // Put button actions here
        RUN_CARGO_ACTIVE_TOP.whileHeld(new CargoActiveCommand(Constants.Physical.CargoActive.SPEED_FWDS, true));
        RUN_CARGO_ACTIVE_BKWDS_TOP.whileHeld(new CargoActiveCommand(Constants.Physical.CargoActive.SPEED_BKWDS, true));

        RUN_CARGO_ACTIVE_BOTTOM.whileHeld(new CargoActiveCommand(Constants.Physical.CargoActive.SPEED_FWDS, false));
        RUN_CARGO_ACTIVE_BKWDS_BOTTOM.whileHeld(new CargoActiveCommand(Constants.Physical.CargoActive.SPEED_BKWDS, false));
    }

    /**
     * Workaround for Java's lazy-loading of static classes
     * <p>
     * When this is called, Java loads the static bits of this class and runs the static init constructor above.
     */
    public static void init() {}

}