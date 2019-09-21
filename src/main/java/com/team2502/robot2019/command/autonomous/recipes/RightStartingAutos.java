package com.team2502.robot2019.command.autonomous.recipes;

import com.github.ezauton.core.action.ActionGroup;
import com.github.ezauton.core.action.DelayedAction;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroAction;
import com.team2502.robot2019.command.autonomous.ingredients.TurnToAnglePDAction;
import com.team2502.robot2019.command.autonomous.ingredients.WaitBasedOnShuffleBoardAction;
import edu.wpi.first.wpilibj.command.Command;

import java.util.concurrent.TimeUnit;

public class RightStartingAutos
{
    public static class Hab2
    {
        public static Command rightNearSideHatch()
        {
            ActionGroup group = new ActionGroup()
                    .addSequential(new WaitBasedOnShuffleBoardAction())
                    .addSequential(new DriveStraightWithGyroAction(-7, 1000))
                    .addSequential(new TimedPeriodicAction(1500, TimeUnit.MILLISECONDS))
                    .addSequential(new TurnToAnglePDAction(2, -Math.PI / 6))
                    .addSequential(new DriveStraightWithGyroAction(-4, 1750))
                    .addSequential(new TurnToAnglePDAction(2, -Math.PI / 2));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }

        public static Command frontRightHatch() {
            ActionGroup group = new ActionGroup()
                    .addSequential(new WaitBasedOnShuffleBoardAction())
                    .addSequential(AutoSpecificUtils.driveOffOfHab2Backwards())
                    .addSequential(new DriveStraightWithGyroAction(-7, 800))
//                    .addSequential(() -> {Robot.DRIVE_TRAIN.runMotorsVelocity(0, 0);})
                    .addSequential(new DelayedAction(3000, TimeUnit.MILLISECONDS))
//                    .addSequential(new TurnToAnglePDAction(8, - Math.PI / 2))
//                    .addSequential(new DriveStraightWithGyroAction(3, 1500))
                    .addSequential(new TurnToAnglePDAction(5, Math.PI));
//                    .addSequential(AutoSpecificUtils.getVisionRoutine());

            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }
    }

    public static class Hab1
    {

    }
}
