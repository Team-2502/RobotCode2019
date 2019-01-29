package com.team2502.robot2019.command.autonomous.ingredients;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;

public class AbortAutoCommand extends InstantCommand
{
    public AbortAutoCommand() { }

    @Override
    protected void execute() { Scheduler.getInstance().removeAll(); }

}
