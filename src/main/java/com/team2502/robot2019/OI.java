package com.team2502.robot2019;

import com.team2502.robot2019.command.LambdaCommand;
import com.team2502.robot2019.command.autonomous.ingredients.AbortAutoCommand;
import com.team2502.robot2019.command.teleop.CargoActive.CargoActiveCommand;
import com.team2502.robot2019.command.teleop.ClimberCommand;
import com.team2502.robot2019.command.teleop.HatchIntakeCommand;
import com.team2502.robot2019.command.teleop.SwitchDriveCommand;
import com.team2502.robot2019.subsystem.CargoSubsystem;
import com.team2502.robot2019.subsystem.ClimberSubsystem;
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

    /**
     * Represents the hatch pusher button
     *
     * @see OI
     */
    public static final Button BUTTON_HATCH_PUSHER = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_HATCH_PUSHER);

    public static final Button BUTTON_ABORT_AUTO = new JoystickButton(JOYSTICK_DRIVE_RIGHT, RobotMap.Joystick.Button.BUTTON_ABORT_AUTO);

    public static final Button RUN_CARGO_ACTIVE_FWD_TOP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.RUN_CARGO_ACTIVE_FWD_TOP);
    public static final Button RUN_CARGO_ACTIVE_BWD_TOP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.RUN_CARGO_ACTIVE_BWD_TOP);

    public static final Button RUN_CARGO_ACTIVE_FWD_BOTTOM = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.RUN_CARGO_ACTIVE_FWD_BOTTOM);
    public static final Button RUN_CARGO_ACTIVE_BWD_BOTTOM = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.RUN_CARGO_ACTIVE_BWD_BOTTOM);

    public static final Button BUTTON_CLIMBER_FWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMBER_FWD);
    public static final Button BUTTON_CLIMBER_BWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMBER_BWD);
    public static final Button BUTTON_CLIMBER_LEFT_FWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMBER_LEFT_FWD);
    public static final Button BUTTON_CLIMBER_LEFT_BWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMBER_LEFT_BWD);
    public static final Button BUTTON_CLIMBER_RIGHT_FWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMBER_RIGHT_FWD);
    public static final Button BUTTON_CLIMBER_RIGHT_BWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMBER_RIGHT_BWD);

    public static final Button CAMERA = new JoystickButton(JOYSTICK_FUNCTION, 2);

    public static final Button SWITCH_DIRECTION = new JoystickButton(JOYSTICK_DRIVE_RIGHT, RobotMap.Joystick.Button.BUTTON_SWITCH_DIRECTION);
    /*
     * Runs when the first static method (usually OI#init()) is called
     * Called the "static initialization constructor"
     */
    static
    {
        BUTTON_HATCH_PUSHER.whenPressed(new HatchIntakeCommand());
        BUTTON_ABORT_AUTO.whenPressed(new AbortAutoCommand());
        SWITCH_DIRECTION.whenPressed(new SwitchDriveCommand());

        // CARGO MANIPULATOR
        RUN_CARGO_ACTIVE_FWD_TOP.whileHeld(new CargoActiveCommand(CargoSubsystem.Belt.TOP, Constants.Physical.CargoActive.SPEED_FWDS));
        RUN_CARGO_ACTIVE_BWD_TOP.whileHeld(new CargoActiveCommand(CargoSubsystem.Belt.TOP, Constants.Physical.CargoActive.SPEED_BKWDS));

        RUN_CARGO_ACTIVE_FWD_BOTTOM.whileHeld(new CargoActiveCommand(CargoSubsystem.Belt.BOTTOM, Constants.Physical.CargoActive.SPEED_FWDS));
        RUN_CARGO_ACTIVE_BWD_BOTTOM.whileHeld(new CargoActiveCommand(CargoSubsystem.Belt.BOTTOM, Constants.Physical.CargoActive.SPEED_BKWDS));



        // CLIMBER
        BUTTON_CLIMBER_FWD.whileHeld(new ClimberCommand(ClimberSubsystem.ClimberSide.BOTH, true));
        BUTTON_CLIMBER_BWD.whileHeld(new ClimberCommand(ClimberSubsystem.ClimberSide.BOTH, false));

        BUTTON_CLIMBER_LEFT_FWD.whileHeld(new ClimberCommand(ClimberSubsystem.ClimberSide.LEFT, true));
        BUTTON_CLIMBER_LEFT_BWD.whileHeld(new ClimberCommand(ClimberSubsystem.ClimberSide.LEFT, false));

        BUTTON_CLIMBER_RIGHT_FWD.whileHeld(new ClimberCommand(ClimberSubsystem.ClimberSide.RIGHT, true));
        BUTTON_CLIMBER_RIGHT_BWD.whileHeld(new ClimberCommand(ClimberSubsystem.ClimberSide.RIGHT, false));

        CAMERA.whenPressed(new LambdaCommand(() -> Robot.SERVER.setSource(Robot.CAMERA1)));
        CAMERA.whenReleased(new LambdaCommand(() -> Robot.SERVER.setSource(Robot.CAMERA0)));
    }

    /**
     * Workaround for Java's lazy-loading of static classes
     * <p>
     * When this is called, Java loads the static bits of this class and runs the static init constructor above.
     */
    public static void init() {}

}