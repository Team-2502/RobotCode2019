package com.team2502.robot2019.command.autonomous.recipes;

import com.github.ezauton.core.action.ActionGroup;
import com.github.ezauton.core.action.BackgroundAction;
import com.github.ezauton.core.action.PurePursuitAction;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.LookaheadBounds;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.recorder.Recording;
import com.github.ezauton.recorder.base.PurePursuitRecorder;
import com.github.ezauton.recorder.base.RobotStateRecorder;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.command.autonomous.ingredients.AccelVelocityDriveAction;
import com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroAction;
import com.team2502.robot2019.command.autonomous.ingredients.SetHatchIntakeAction;
import com.team2502.robot2019.command.autonomous.ingredients.TurnToAnglePDAction;
import com.team2502.robot2019.command.vision.GoToTargetNTAction;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

public class CenterStartingAutos
{

    public static Command frontRightHatch()
    {
        ActionGroup group = new ActionGroup()
                .addSequential(new SetHatchIntakeAction(false))
                .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                .addSequential(new TimedPeriodicAction(250, TimeUnit.MILLISECONDS))

                .addSequential(new TurnToAnglePDAction(2, -15 * Math.PI / 180))
                .addSequential(new DriveStraightWithGyroAction(2.5, 750))
                .addSequential(new TurnToAnglePDAction(2, 0))

                .addSequential(new DriveStraightWithGyroAction(2, 700))
                .addSequential(AutoSpecificUtils.getVisionRoutine())
                .addSequential(new AccelVelocityDriveAction(-4, -4, 0.25, 1000));
        return new CommandCreator(group, Robot.ACTION_SCHEDULER);
    }

    public static Command frontLeftHatch()
    {
            ActionGroup group = new ActionGroup()
                    .addSequential(new SetHatchIntakeAction(false))
                    .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                    .addSequential(new TimedPeriodicAction(250, TimeUnit.MILLISECONDS))
                    .addSequential(new TurnToAnglePDAction(2, 15 * Math.PI / 180))
                    .addSequential(new DriveStraightWithGyroAction(2.5, 1000))
                    .addSequential(new TurnToAnglePDAction(2, 15 * .4 * Math.PI / 180))
                    .addSequential(new DriveStraightWithGyroAction(2, 500))
                    .addSequential(AutoSpecificUtils.getVisionRoutine())
                    .addSequential(new AccelVelocityDriveAction(-4, -4, 0.25, 1000));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
    }

    public static Command rightNearSideHatch()
    {
        ActionGroup group = new ActionGroup()
                .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                .addSequential(new DriveStraightWithGyroAction(2.5, (long) (4500), -Math.PI / 5))
                .addSequential(new TurnToAnglePDAction(1, Math.PI / 2 - Math.PI / 6))
                .addSequential(new GoToTargetNTAction())
                .addSequential(new DriveStraightWithGyroAction(4, 1000))
                .addSequential(new SetHatchIntakeAction(true))
                .addSequential(new TimedPeriodicAction(400, TimeUnit.MILLISECONDS))
                .addSequential(new DriveStraightWithGyroAction(-1, 1000));
        return new CommandCreator(group, Robot.ACTION_SCHEDULER);
    }

    public static Command rightNearSideHatchPP()
    {

        Path path = new SplinePPWaypoint.Builder(4)
                .add(0, 0, 0, 2, 13, -12)
                .add(0, 6, 0, 10, 10, 13, -13)
//                        .add(6, 10, 2.5, 1.25, 6, 13, -12)
                .add(5.155-1.5, 17.265+1.5, 0, 25, 3, 13, -12)
//                        .add(3.93, 17.3, -10, 0, 6, 13, -13)

//                        .add(6.6, 14, 10, 10, 2, 13, -12)
//                        .add(3, 17.387, -0.2, 0, 2, 13, -12)
//                        .add(5, 17.2, -0.001, 0, 3.5, 13, -12)
                .buildPathGenerator()
                .generate(0.05);

        PurePursuitMovementStrategy ppms = new PurePursuitMovementStrategy(path, 3 / 12D);
        Recording rec = new Recording()
                .addSubRecording(new RobotStateRecorder(RealClock.CLOCK, Robot.DRIVE_TRAIN.getLocEstimator(), Robot.DRIVE_TRAIN.getRotEstimator(), 30D / 12, 36D / 12))
                .addSubRecording(new PurePursuitRecorder(RealClock.CLOCK, path, ppms));

        Robot.onDisableThings.add(() -> {
            try
            {
                rec.save("applebanana.json");
            }
            catch(IOException e)
            {
                e.printStackTrace();
            }
        });

        ActionGroup group = new ActionGroup()
                .addParallel(new BackgroundAction(20, TimeUnit.MILLISECONDS, rec::update))
                .addSequential(new SetHatchIntakeAction(false))
                .addSequential(new PurePursuitAction(20, TimeUnit.MILLISECONDS, ppms, Robot.DRIVE_TRAIN.getLocEstimator(),
                        new LookaheadBounds(
                                SmartDashboard.getNumber("minDistance", 1),
                                SmartDashboard.getNumber("maxDistance", 5),
                                SmartDashboard.getNumber("minSpeed", 3),
                                SmartDashboard.getNumber("maxSpeed", 10),
                                Robot.DRIVE_TRAIN.getVelocityEstimator()
                        )
                        , Robot.DRIVE_TRAIN))
                .addParallel(new TurnToAnglePDAction(2, Math.PI / 2))
                .addSequential(AutoSpecificUtils.getVisionRoutine())
                .addSequential(new SetHatchIntakeAction(true))
                .addSequential(new DriveStraightWithGyroAction(-3, 1000));

        return new CommandCreator(group, Robot.ACTION_SCHEDULER);
    }

    public static Command leftNearSideHatchPP()
    {

        Path path = new SplinePPWaypoint.Builder(4)
                .add(0, 0, 0, 2, 13, -12)
                .add(0, 6, 0, 10, 10, 13, -13)
//                        .add(6, 10, 2.5, 1.25, 6, 13, -12)
                .add(5.155-1.5, 17.265+1.5, 0, 25, 3, 13, -12)
//                        .add(3.93, 17.3, -10, 0, 6, 13, -13)

//                        .add(6.6, 14, 10, 10, 2, 13, -12)
//                        .add(3, 17.387, -0.2, 0, 2, 13, -12)
//                        .add(5, 17.2, -0.001, 0, 3.5, 13, -12)
                .flipY()
                .buildPathGenerator()
                .generate(0.05);

        PurePursuitMovementStrategy ppms = new PurePursuitMovementStrategy(path, 3 / 12D);
        Recording rec = new Recording()
                .addSubRecording(new RobotStateRecorder(RealClock.CLOCK, Robot.DRIVE_TRAIN.getLocEstimator(), Robot.DRIVE_TRAIN.getRotEstimator(), 30D / 12, 36D / 12))
                .addSubRecording(new PurePursuitRecorder(RealClock.CLOCK, path, ppms));

        Robot.onDisableThings.add(() -> {
            try
            {
                rec.save("applebananaleft.json");
            }
            catch(IOException e)
            {
                e.printStackTrace();
            }
        });

        ActionGroup group = new ActionGroup()
                .addParallel(new BackgroundAction(20, TimeUnit.MILLISECONDS, rec::update))
                .addSequential(new SetHatchIntakeAction(false))
                .addSequential(new PurePursuitAction(20, TimeUnit.MILLISECONDS, ppms, Robot.DRIVE_TRAIN.getLocEstimator(),
                                                     new LookaheadBounds(
                                                             SmartDashboard.getNumber("minDistance", 1),
                                                             SmartDashboard.getNumber("maxDistance", 5),
                                                             SmartDashboard.getNumber("minSpeed", 3),
                                                             SmartDashboard.getNumber("maxSpeed", 10),
                                                             Robot.DRIVE_TRAIN.getVelocityEstimator()
                                                     )
                        , Robot.DRIVE_TRAIN))
                .addParallel(new TurnToAnglePDAction(2, Math.PI / 2))
                .addSequential(AutoSpecificUtils.getVisionRoutine())
                .addSequential(new SetHatchIntakeAction(true))
                .addSequential(new DriveStraightWithGyroAction(-3, 1000));

        return new CommandCreator(group, Robot.ACTION_SCHEDULER);
    }
}
