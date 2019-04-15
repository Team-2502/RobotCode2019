package com.team2502.robot2019;


import com.github.ezauton.core.action.*;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.team2502.robot2019.command.autonomous.ingredients.*;
import com.team2502.robot2019.command.vision.GoToTargetNTAction;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
            Path path =new SplinePPWaypoint.Builder()
                    .add(0, 0, 0, 2, 13, -12)
                    .add(0, 3, -Math.PI/2, 2, 13, -13)
                    .add(3, 3, 0, 2, 13, -12)
                    .add(3, 6, 0, 0, 13, -12)
                    .buildPathGenerator()
                    .generate(0.05);

            PurePursuitMovementStrategy ppMoveStrat = new PurePursuitMovementStrategy(path, 0.1);
            PurePursuitAction pp = new PurePursuitAction(20, TimeUnit.MILLISECONDS, ppMoveStrat, Robot.DRIVE_TRAIN.getLocEstimator(), Constants.Autonomous.getLookaheadBounds(Robot.DRIVE_TRAIN), Robot.DRIVE_TRAIN);
            ActionGroup group = new ActionGroup().addSequential(() -> {
                try
                {
                    Robot.DRIVE_TRAIN.take();
                }
                catch(InterruptedException e)
                {
                    e.printStackTrace();
                }
            }).addSequential(pp).addSequential(Robot.DRIVE_TRAIN::giveBack);
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        ACTION_GROUP_PARALLEL_TEST("PP Action Group Parallel Test", () -> {
            ActionGroup group = new ActionGroup();
            group.with(new BackgroundAction(10, TimeUnit.MILLISECONDS, Robot.DRIVE_TRAIN::update));
            group.addSequential((Action) new VoltageDriveAction(0.3, 0.3, 3));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        VISION_TEST("VisionTest", () -> {
            ActionGroup group = new ActionGroup()
                    .addSequential(new GoToTargetNTAction())
                    .addSequential(new DriveStraightWithGyroAction(4, 2000));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        DRIVE_STRAIGHT_TEST("drive straight", () -> {
            ActionGroup group = new ActionGroup()
                    .addSequential(new DriveStraightWithGyroAction(1, 50000));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        FRONT_LEFT_HATCH("MID Hab -> LEFT Front Hatch", () -> {
            ActionGroup group = new ActionGroup()
                    .addSequential(new SetHatchIntakeAction(false))
                    .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                    .addSequential(new TimedPeriodicAction(250, TimeUnit.MILLISECONDS))
                    .addSequential(new TurnToAnglePDAction(2, 15 * Math.PI / 180))
                    .addSequential(new DriveStraightWithGyroAction(2.5, 1000))
                    .addSequential(new TurnToAnglePDAction(2, 15 * .4 * Math.PI / 180))
                    .addSequential(getVisionRoutine())
                    .addSequential(new AccelVelocityDriveAction(-4, -4, 0.25, 1000));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),

        FRONT_RIGHT_HATCH("MID Hab -> RIGHT Front Hatch", () -> {
        ActionGroup group = new ActionGroup()
                .addSequential(new SetHatchIntakeAction(false))
                .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                .addSequential(new TimedPeriodicAction(250, TimeUnit.MILLISECONDS))
                .addSequential(new TurnToAnglePDAction(2, -15 * Math.PI / 180))
                .addSequential(new DriveStraightWithGyroAction(2.5, 1000))
                .addSequential(new TurnToAnglePDAction(2, -15 * .4 * Math.PI / 180))
                .addSequential(getVisionRoutine())
                .addSequential(new AccelVelocityDriveAction(-4, -4, 0.25, 1000));
        return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        NEARSIDE_RIGHT_HATCH("MID Hab -> RIGHT nearSIDE Hatch", () -> {
            ActionGroup group = new ActionGroup()
                    .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                    .addSequential(new DriveStraightWithGyroAction(2.5, (long) (4500), -Math.PI / 5))
                    .addSequential(new TurnToAnglePDAction(1, Math.PI/2 - Math.PI / 6))
                    .addSequential(new GoToTargetNTAction())
                    .addSequential(new DriveStraightWithGyroAction(4, 1000))
                    .addSequential(new SetHatchIntakeAction(true))
                    .addSequential(new TimedPeriodicAction(400, TimeUnit.MILLISECONDS))
                    .addSequential(new DriveStraightWithGyroAction(-1, 1000));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        NEARSIDE_RIGHT_HATCH_PP("MID Hab -> RIGHT nearSIDE Hatch", () -> {
            ActionGroup group = new ActionGroup()
                    .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                    .addSequential(new DriveStraightWithGyroAction(2.5, (long) (4500), -Math.PI / 5))
                    .addSequential(new TurnToAnglePDAction(1, Math.PI/2))
                    .addSequential(new GoToTargetNTAction())
                    .addSequential(new DriveStraightWithGyroAction(4, 1000))
                    .addSequential(new SetHatchIntakeAction(true))
                    .addSequential(new TimedPeriodicAction(400, TimeUnit.MILLISECONDS))
                    .addSequential(new DriveStraightWithGyroAction(-1, 1000));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        RIGHT_NEAR_HATCH_HAB2("RIGHT Hab 2 -> Right nearSIDE Hatch CARGO SHIP", () -> {
            ActionGroup group = new ActionGroup()
                    .addSequential(new DriveStraightWithGyroAction(-7, 1000))
                    .addSequential(new TimedPeriodicAction(1500, TimeUnit.MILLISECONDS))
                    .addSequential(new TurnToAnglePDAction(2, -Math.PI / 6))
                    .addSequential(new DriveStraightWithGyroAction(-4, 1750))
                    .addSequential(new TurnToAnglePDAction(2, -Math.PI/2));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        LEFT_NEAR_HATCH_HAB2("LEFT Hab 2 -> LEFT nearSIDE Hatch CARGO SHIP", () -> {
            ActionGroup group = new ActionGroup()
                    .addSequential(new DriveStraightWithGyroAction(-7, 1000))
                    .addSequential(new TimedPeriodicAction(1500, TimeUnit.MILLISECONDS))
                    .addSequential(new TurnToAnglePDAction(2, Math.PI / 6))
                    .addSequential(new DriveStraightWithGyroAction(-4, 1750))
                    .addSequential(new TurnToAnglePDAction(2, Math.PI/2));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }),
        TURNYTEST("turn test", () -> {
            return new CommandCreator(new TurnToAnglePDAction(1, Math.PI / 2), Robot.ACTION_SCHEDULER);
        }),
        ACCELY_TEST("drivey test", () -> {
            return new CommandCreator(new MotionMagicDriveAction(10, 10, 10, 10), Robot.ACTION_SCHEDULER);
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

    /**
     * @return An action group that will do vision, deploy hatch mech, and wait
     */
    private static ActionGroup getVisionRoutine()  {
        return new ActionGroup().addSequential(new GoToTargetNTAction())
                .addSequential(new DriveStraightWithGyroAction(3, 1000))
                .addSequential(new TimedPeriodicAction(400, TimeUnit.MILLISECONDS))
                .addSequential(new SetHatchIntakeAction(true))
                .addSequential(new TimedPeriodicAction(400, TimeUnit.MILLISECONDS));

    }

}
