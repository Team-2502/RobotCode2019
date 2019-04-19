package com.team2502.robot2019.command.autonomous.recipes;

import com.github.ezauton.core.action.ActionGroup;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.team2502.robot2019.command.autonomous.ingredients.DriveStraightWithGyroAction;
import com.team2502.robot2019.command.autonomous.ingredients.SetHatchIntakeAction;
import com.team2502.robot2019.command.vision.GoToTargetNTAction;

import java.util.concurrent.TimeUnit;

public class AutoSpecificUtils
{
    /**
     * @return An action group that will do vision, deploy hatch mech, and wait
     */
    public static ActionGroup getVisionRoutine()
    {
        return new ActionGroup().addSequential(new GoToTargetNTAction())
                                .addSequential(new DriveStraightWithGyroAction(3, 1000))
                                .addSequential(new TimedPeriodicAction(400, TimeUnit.MILLISECONDS))
                                .addSequential(new SetHatchIntakeAction(true))
                                .addSequential(new TimedPeriodicAction(400, TimeUnit.MILLISECONDS));

    }
}
