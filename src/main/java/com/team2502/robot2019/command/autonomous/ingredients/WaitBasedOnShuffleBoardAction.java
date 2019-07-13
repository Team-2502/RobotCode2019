package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.PeriodicAction;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;

public class WaitBasedOnShuffleBoardAction extends PeriodicAction
{

    private double delayTimeMillis = 0;

    @Override
    protected void init() throws Exception
    {
        stopwatch.reset();
    }

    @Override
    protected void execute() throws Exception
    {
        // allows for drivers to go "oh crap! it's too low/high!" and adjust during action
        delayTimeMillis = SmartDashboard.getNumber(Constants.ShuffleboardKeys.DELAY_AUTO, 0) * 1000;
    }

    @Override
    protected boolean isFinished() throws Exception
    {
        return true;
//        return Robot.okToStartAuto;
    }
}
