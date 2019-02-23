package com.team2502.robot2019;


import com.github.ezauton.core.action.*;
import com.github.ezauton.core.pathplanning.PP_PathGenerator;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.PPWaypoint;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.team2502.robot2019.command.autonomous.ingredients.DoNothingCommand;
import com.team2502.robot2019.command.autonomous.ingredients.PointDriveAction;
import com.team2502.robot2019.command.autonomous.ingredients.VelocityDriveAction;
import com.team2502.robot2019.command.autonomous.ingredients.VoltageDriveAction;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.*;
import java.util.concurrent.TimeUnit;

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
        DO_NOTHING("Do Nothing", DoNothingCommand::new),
        ACTION_GROUP_TEST("PP Action Group Test", () -> {
            ActionGroup group = new ActionGroup();
            PPWaypoint[] waypoints = new PPWaypoint.Builder()
                    .add(0, 0, 1600, 130000, -120000)
                    .add(0, 4, 0, 130000, -120000)
                    .buildArray();
            PP_PathGenerator pathGenerator = new PP_PathGenerator(waypoints);
            Path path = pathGenerator.generate(0.05);

            PurePursuitMovementStrategy ppMoveStrat = new PurePursuitMovementStrategy(path, 0.001);
            PurePursuitAction pp = new PurePursuitAction(10, TimeUnit.MILLISECONDS, ppMoveStrat, Robot.DRIVE_TRAIN.getLocEstimator(), Constants.Autonomous.getLookaheadBounds(Robot.DRIVE_TRAIN), Robot.DRIVE_TRAIN);

            group.with(new BackgroundAction(10, TimeUnit.MILLISECONDS, Robot.DRIVE_TRAIN::update));
            group.addSequential(pp);//new PointDriveAction(.05, new ImmutableVector(3, 10), 3));
            group.addSequential((IAction) new VoltageDriveAction(-0.3, -0.3, 3));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        ACTION_GROUP_PARALLEL_TEST("PP Action Group Parallel Test", () -> {
            ActionGroup group = new ActionGroup();
            group.with(new BackgroundAction(10, TimeUnit.MILLISECONDS, Robot.DRIVE_TRAIN::update));
            group.addSequential((IAction) new VoltageDriveAction(0.3, 0.3, 3));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        });


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
