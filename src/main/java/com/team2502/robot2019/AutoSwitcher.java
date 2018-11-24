package com.team2502.robot2019;


import com.team2502.robot2019.command.autonomous.ingredients.DoNothingCommand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Select where we start the robot so it knows where it is on the field, resulting in a successful autonomous routine.
 */
public class AutoSwitcher
{
    /**
     * The actual sendable containing the autonomi
     */
    private static SendableChooser<AutoMode> autoChooser;

    /**
     * Initialize AutoStartLocationSwitcher#autoChooser, put the enum values in it, and put it on the dashboard
     */
    static void putToSmartDashboard()
    {
        autoChooser = new SendableChooser<>();

        for(int i = 0; i < AutoMode.values().length; i++)
        {
            AutoMode mode = AutoMode.values()[i];
            if(i == 0) { autoChooser.addDefault(mode.name, mode); }
            else { autoChooser.addObject(mode.name, mode); }
        }

        SmartDashboard.putData("auto_modes", autoChooser);
    }

    /**
     * Get an instance of the autonomous selected
     *
     * @return A new instance of the selected autonomous
     */
    static Command getAutoInstance() { return autoChooser.getSelected().getInstance(); }

    public static AutoMode getSelectedPosition() { return autoChooser.getSelected(); }
    /**
     * An enum containing all the autonomi the drivers can select from
     */
    public enum AutoMode
    {
        DO_NOTHING("Do Nothing", DoNothingCommand::new);

        /**
         * A lambda that creates a new instance of the command
         */
        public final CommandFactory commandFactory;

        /**
         * The name of the command to display on the driver station
         */
        public final String name;

        /**
         * Make a new auto mode that can be selected from
         *
         * @param name           The name of the command
         * @param commandFactory A lambda that can create a new command (usually method reference to constructor)
         */
        AutoMode(String name, CommandFactory commandFactory)
        {
            this.commandFactory = commandFactory;
            this.name = name;
        }

        /**
         * @return A new instance of the command (generally runs constructor)
         */
        public Command getInstance()
        {
            return commandFactory.getInstance();
        }
    }

    @FunctionalInterface
    public interface CommandFactory
    {
        /**
         * Create a command
         * @return A new command, fresh from the factory
         */
        Command getInstance();
    }

}