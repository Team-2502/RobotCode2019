package com.team2502.robot2019;

import com.team2502.robot2019.command.LambdaCommand;
import com.team2502.robot2019.command.autonomous.ingredients.AbortAutoCommand;
import com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroCommand;
import com.team2502.robot2019.command.autonomous.ingredients.VelocityDriveCommand;
import com.team2502.robot2019.command.teleop.cargoactive.CargoActiveCommand;
import com.team2502.robot2019.command.teleop.cargoactive.ToggleOBACommand;
import com.team2502.robot2019.command.teleop.climber.ClimbClawCommand;
import com.team2502.robot2019.command.teleop.climber.ClimbCommand;
import com.team2502.robot2019.command.teleop.climber.CrawlCommand;
import com.team2502.robot2019.command.teleop.HatchIntakeCommand;

import com.team2502.robot2019.command.vision.GoToTargetLimelightSimple;
import com.team2502.robot2019.subsystem.CargoSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;


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

    /**
     * Represents the side button panel
     *
     * @see OI
     */
    public static final Joystick JOYSTICK_SIDE_PANEL = new Joystick(RobotMap.Joystick.JOYSTICK_SIDE_PANEL);

    // Start defining buttons to be using
    // Names are self explanatory
    // Convention: Button variable names here should be the same as ID names in RobotMap

    /**
     * Represents the hatch pusher button
     *
     * @see OI
     */
    public static final Button BUTTON_HATCH_PUSHER = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_HATCH_PUSHER);

    public static final Button BUTTON_ENABLE_AUTO_ALIGN = new JoystickButton(JOYSTICK_DRIVE_RIGHT, RobotMap.Joystick.Button.BUTTON_ENABLE_AUTO_ALIGN);

    public static final Button BUTTON_RUN_CARGO_ACTIVE_FWD_TOP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RUN_CARGO_ACTIVE_FWD_TOP);
    public static final Button BUTTON_RUN_CARGO_ACTIVE_BWD_TOP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RUN_CARGO_ACTIVE_BWD_TOP);

    public static final Button BUTTON_RUN_CARGO_OBA_FWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RUN_CARGO_ACTIVE_FWD_BOTTOM);
    public static final Button BUTTON_RUN_CARGO_OBA_BWD = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RUN_CARGO_ACTIVE_BWD_BOTTOM);

    public static final Button BUTTON_CLIMB_UP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMBER_STRUCTURE_UP);
    public static final Button BUTTON_CLIMB_DOWN = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_CLIMBER_STRUCTURE_DOWN);

    public static final Button BUTTON_CLIMB_RIGHT_UP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RIGHT_CLIMB_UP);
    public static final Button BUTTON_CLIMB_RIGHT_DOWN = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_RIGHT_CLIMB_DOWN);
    public static final Button BUTTON_CLIMB_LEFT_UP = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_LEFT_CLIMB_UP);
    public static final Button BUTTON_CLIMB_LEFT_DOWN = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_LEFT_CLIMB_DOWN);

    public static final Button BUTTON_TOGGLE_FLIP_OUT = new JoystickButton(JOYSTICK_SIDE_PANEL, RobotMap.Joystick.Button.BUTTON_TOGGLE_FLIP_OUT);
    public static final Button BUTTON_DRIVER_FLIP_OUT_LEFT = new JoystickButton(JOYSTICK_DRIVE_LEFT, RobotMap.Joystick.Button.BUTTON_DRIVER_ANTI_TIP);
    public static final Button BUTTON_DRIVER_FLIP_OUT_RIGHT = new JoystickButton(JOYSTICK_DRIVE_RIGHT, RobotMap.Joystick.Button.BUTTON_DRIVER_ANTI_TIP);

    public static final Button BUTTON_TOGGLE_OBA = new JoystickButton(JOYSTICK_FUNCTION, RobotMap.Joystick.Button.BUTTON_SWITCH_CAMERA);


    public static final Button BUTTON_CRAWL = new JoystickButton(JOYSTICK_DRIVE_LEFT, RobotMap.Joystick.Button.BUTTON_CRAWL);


    public static final Button BUTTON_DRIVE_FORWARDS = new JoystickButton(JOYSTICK_DRIVE_LEFT, 8);
    public static final Button BUTTON_DRIVE_FORWARDS_NOGYRO = new JoystickButton(JOYSTICK_DRIVE_LEFT, 7);

    public static final Button BUTTON_ABORT_AUTO = new JoystickButton(JOYSTICK_SIDE_PANEL, 6);
    public static final Button BUTTON_KILL_ACTIONS = new JoystickButton(JOYSTICK_DRIVE_LEFT, RobotMap.Joystick.Button.BUTTON_KILL_ACTIONS);
    public static int camera1Selected = 0;
    /*
     * Runs when the first static method (usually OI#init()) is called
     * Called the "static initialization constructor"
     */
    static
    {
        BUTTON_HATCH_PUSHER.whenPressed(new HatchIntakeCommand());
        BUTTON_ENABLE_AUTO_ALIGN.whenPressed(new GoToTargetLimelightSimple());
        BUTTON_ENABLE_AUTO_ALIGN.whenReleased(new AbortAutoCommand());

        BUTTON_ABORT_AUTO.whenPressed(new LambdaCommand(() -> {
            Scheduler.getInstance().removeAll();
            try
            {
                Robot.ACTION_SCHEDULER.killAll();
            }
            catch(Exception e)
            {
                e.printStackTrace();
            }
        }));

        // CARGO MANIPULATOR
        BUTTON_RUN_CARGO_ACTIVE_FWD_TOP.whileHeld(new CargoActiveCommand(CargoSubsystem.Section.INTERNAL, Constants.Physical.CargoActive.SPEED_FWD));
        BUTTON_RUN_CARGO_ACTIVE_BWD_TOP.whileHeld(new CargoActiveCommand(CargoSubsystem.Section.INTERNAL, Constants.Physical.CargoActive.SPEED_BWD));

        BUTTON_RUN_CARGO_OBA_FWD.whileHeld(new CargoActiveCommand(CargoSubsystem.Section.BOTH, Constants.Physical.OverBumperActive.SPEED_FWD));
        BUTTON_RUN_CARGO_OBA_BWD.whileHeld(new CargoActiveCommand(CargoSubsystem.Section.BOTH, Constants.Physical.OverBumperActive.SPEED_BWD));

        // CLIMBER
        BUTTON_CLIMB_UP.whileHeld(new ClimbCommand(true, ClimbCommand.Side.BOTH));
        BUTTON_CLIMB_DOWN.whileHeld(new ClimbCommand(false, ClimbCommand.Side.BOTH));

        BUTTON_CLIMB_RIGHT_UP.whileHeld(new ClimbCommand(true, ClimbCommand.Side.RIGHT));
        BUTTON_CLIMB_RIGHT_DOWN.whileHeld(new ClimbCommand(false, ClimbCommand.Side.RIGHT));

        BUTTON_CLIMB_LEFT_UP.whileHeld(new ClimbCommand(true, ClimbCommand.Side.LEFT));
        BUTTON_CLIMB_LEFT_DOWN.whileHeld(new ClimbCommand(false, ClimbCommand.Side.LEFT));

        BUTTON_CRAWL.whileHeld(new CrawlCommand());
        BUTTON_TOGGLE_FLIP_OUT.whenPressed(new ClimbClawCommand());

        BUTTON_TOGGLE_OBA.whenPressed(new ToggleOBACommand());

        BUTTON_DRIVER_FLIP_OUT_LEFT.whenPressed(new ClimbClawCommand());
        BUTTON_DRIVER_FLIP_OUT_RIGHT.whenPressed(new ClimbClawCommand());

        BUTTON_DRIVE_FORWARDS_NOGYRO.whileHeld(new VelocityDriveCommand(4, 4, 4));

        BUTTON_DRIVE_FORWARDS.whileHeld(new DriveStraightWithGyroCommand(2,5));

        BUTTON_KILL_ACTIONS.whenPressed(new LambdaCommand(() -> {
            Scheduler.getInstance().removeAll();
            try
            {
                Robot.ACTION_SCHEDULER.killAll();
            }
            catch(Exception e)
            {
                e.printStackTrace();
            }
        }));
    }

    /**
     * Workaround for Java's lazy-loading of static classes
     * <p>
     * When this is called, Java loads the static bits of this class and runs the static init constructor above.
     */
    public static void init() {}

}
