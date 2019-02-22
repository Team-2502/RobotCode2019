package com.team2502.robot2019;

import com.team2502.robot2019.command.LambdaCommand;
import com.team2502.robot2019.command.autonomous.ingredients.AbortAutoCommand;
import com.team2502.robot2019.command.teleop.cargoactive.CargoActiveCommand;
import com.team2502.robot2019.command.teleop.climber.ClimbCommand;
import com.team2502.robot2019.command.teleop.climber.CrawlCommand;
import com.team2502.robot2019.command.teleop.HatchIntakeCommand;
import com.team2502.robot2019.command.teleop.SwitchDriveCommand;
import com.team2502.robot2019.subsystem.CargoSubsystem;
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

    public static final Button BUTTON_RUN_CARGO_ACTIVE_FWD_TOP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RUN_CARGO_ACTIVE_FWD_TOP);
    public static final Button BUTTON_RUN_CARGO_ACTIVE_BWD_TOP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RUN_CARGO_ACTIVE_BWD_TOP);

    public static final Button BUTTON_RUN_CARGO_ACTIVE_FWD_BOTTOM = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RUN_CARGO_ACTIVE_FWD_BOTTOM);
    public static final Button BUTTON_RUN_CARGO_ACTIVE_BWD_BOTTOM = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RUN_CARGO_ACTIVE_BWD_BOTTOM);

    public static final Button BUTTON_CLIMB_UP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMB_UP);
    public static final Button BUTTON_CLIMB_DOWN = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMB_DOWN);
    public static final Button BUTTON_CRAWL_FWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CRAWL_FWD);
    public static final Button BUTTON_CRAWL_BWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CRAWL_BWD);

    public static final Button BUTTON_SWITCH_CAMERA = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_SWITCH_CAMERA);

    public static final Button BUTTON_SWITCH_DIRECTION = new JoystickButton(JOYSTICK_DRIVE_RIGHT, RobotMap.Joystick.Button.BUTTON_SWITCH_DIRECTION);

    public static int camera1Selected = 0;
    /*
     * Runs when the first static method (usually OI#init()) is called
     * Called the "static initialization constructor"
     */
    static
    {
        BUTTON_HATCH_PUSHER.whenPressed(new HatchIntakeCommand());
        BUTTON_ABORT_AUTO.whenPressed(new AbortAutoCommand());
        BUTTON_SWITCH_DIRECTION.whenPressed(new SwitchDriveCommand());

        // CARGO MANIPULATOR
        BUTTON_RUN_CARGO_ACTIVE_FWD_TOP.whileHeld(new CargoActiveCommand(CargoSubsystem.Belt.TOP, Constants.Physical.CargoActive.SPEED_FWDS));
        BUTTON_RUN_CARGO_ACTIVE_BWD_TOP.whileHeld(new CargoActiveCommand(CargoSubsystem.Belt.TOP, Constants.Physical.CargoActive.SPEED_BKWDS));

        BUTTON_RUN_CARGO_ACTIVE_FWD_BOTTOM.whileHeld(new CargoActiveCommand(CargoSubsystem.Belt.BOTTOM, Constants.Physical.CargoActive.SPEED_FWDS));
        BUTTON_RUN_CARGO_ACTIVE_BWD_BOTTOM.whileHeld(new CargoActiveCommand(CargoSubsystem.Belt.BOTTOM, Constants.Physical.CargoActive.SPEED_BKWDS));


        // CLIMBER
        BUTTON_CLIMB_UP.whileHeld(new ClimbCommand(true));
        BUTTON_CLIMB_DOWN.whileHeld(new ClimbCommand(false));

        BUTTON_CRAWL_FWD.whileHeld(new CrawlCommand(true));
        BUTTON_CRAWL_BWD.whileHeld(new CrawlCommand(false));


        // BUTTON_SWITCH_CAMERA
        BUTTON_SWITCH_CAMERA.whenPressed(new LambdaCommand(() -> {
            switch (camera1Selected = camera1Selected % 3)
            {
                case 0:
                    Robot.SERVER.setSource(Robot.CAMERA0);
                    break;
                case 1:
                    Robot.SERVER.setSource(Robot.CAMERA1);
                    break;
                case 2:
                    Robot.SERVER.setSource(Robot.CAMERA2);
                    break;
            }
            camera1Selected++;
        }));
    }

    /**
     * Workaround for Java's lazy-loading of static classes
     * <p>
     * When this is called, Java loads the static bits of this class and runs the static init constructor above.
     */
    public static void init() {}

}
