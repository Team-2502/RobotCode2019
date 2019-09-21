package com.team2502.robot2019.command.autonomous.recipes;

import com.github.ezauton.core.action.ActionGroup;
import com.github.ezauton.core.action.BackgroundAction;
import com.github.ezauton.core.action.DelayedAction;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.localization.RotationalLocationEstimator;
import com.github.ezauton.core.localization.TranslationalLocationEstimator;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.recorder.Recording;
import com.github.ezauton.recorder.base.RobotStateRecorder;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroAction;
import com.team2502.robot2019.command.autonomous.ingredients.TurnToAnglePDAction;
import com.team2502.robot2019.command.autonomous.ingredients.WaitBasedOnShuffleBoardAction;
import edu.wpi.first.wpilibj.command.Command;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

public class LeftStartingAutos
{
    public static class Hab2 {
        public static Command leftNearSideHatch() {
            ActionGroup group = new ActionGroup()
                    .addSequential(new WaitBasedOnShuffleBoardAction())
                    .addSequential(new DriveStraightWithGyroAction(-7, 1000))
                    .addSequential(new TimedPeriodicAction(1500, TimeUnit.MILLISECONDS))
                    .addSequential(new TurnToAnglePDAction(2, Math.PI / 6))
                    .addSequential(new DriveStraightWithGyroAction(-4, 1750))
                    .addSequential(new TurnToAnglePDAction(2, Math.PI/2));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }

        public static Command frontLeftHatch() {


            ActionGroup group = new ActionGroup()
                    .addSequential(new WaitBasedOnShuffleBoardAction())
                    .addSequential(AutoSpecificUtils.driveOffOfHab2Backwards())
                    .addSequential(new DriveStraightWithGyroAction(-3, 2000, -Math.PI / 2))
                    .addSequential(new TurnToAnglePDAction(5, -Math.PI));
//                    .addSequential(AutoSpecificUtils.getVisionRoutine());

            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }
    }

    public static class Hab1 {

    }
}
