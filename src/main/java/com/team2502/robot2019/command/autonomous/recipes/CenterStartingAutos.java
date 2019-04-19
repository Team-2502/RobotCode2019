package com.team2502.robot2019.command.autonomous.recipes;

import com.github.ezauton.core.action.ActionGroup;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.command.autonomous.ingredients.AccelVelocityDriveAction;
import com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroAction;
import com.team2502.robot2019.command.autonomous.ingredients.SetHatchIntakeAction;
import com.team2502.robot2019.command.autonomous.ingredients.TurnToAnglePDAction;
import com.team2502.robot2019.command.vision.GoToTargetNTAction;
import edu.wpi.first.wpilibj.command.Command;

import java.util.concurrent.TimeUnit;

public class CenterStartingAutos
{
    public static Command frontRightHatch() {
        ActionGroup group = new ActionGroup()
                .addSequential(new SetHatchIntakeAction(false))
                .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                .addSequential(new TimedPeriodicAction(250, TimeUnit.MILLISECONDS))
                .addSequential(new TurnToAnglePDAction(2, -15 * Math.PI / 180))
                .addSequential(new DriveStraightWithGyroAction(2.5, 1000))
                .addSequential(new TurnToAnglePDAction(2, 15 * .4 * Math.PI / 180))
                .addSequential(new DriveStraightWithGyroAction(3, 1000))

                .addSequential(AutoSpecificUtils.getVisionRoutine());
//                .addSequential(new AccelVelocityDriveAction(-4, -4, 0.25, 1000));
        return new CommandCreator(group, Robot.ACTION_SCHEDULER);
    }

    public static Command frontLeftHatch() {
        {
            ActionGroup group = new ActionGroup()
                    .addSequential(new SetHatchIntakeAction(false))
                    .addSequential(new DriveStraightWithGyroAction(2.5, 1500))
                    .addSequential(new TimedPeriodicAction(250, TimeUnit.MILLISECONDS))
                    .addSequential(new TurnToAnglePDAction(2, 15 * Math.PI / 180))
                    .addSequential(new DriveStraightWithGyroAction(2.5, 1000))
                    .addSequential(new TurnToAnglePDAction(2, 15 * .4 * Math.PI / 180))
                    .addSequential(AutoSpecificUtils.getVisionRoutine())
                    .addSequential(new AccelVelocityDriveAction(-4, -4, 0.25, 1000));
            return new CommandCreator(group, Robot.ACTION_SCHEDULER);
        }
    }

    public static Command rightNearSideHatch() {
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
    }


}
