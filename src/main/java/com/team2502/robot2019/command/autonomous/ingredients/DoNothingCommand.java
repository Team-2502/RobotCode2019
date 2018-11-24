package com.team2502.robot2019.command.autonomous.ingredients;

import edu.wpi.first.wpilibj.command.Command;

public class DoNothingCommand extends Command
{
    @Override
    protected boolean isFinished()
    {
        return true; // Finished instantly, does nothing
    }
}
