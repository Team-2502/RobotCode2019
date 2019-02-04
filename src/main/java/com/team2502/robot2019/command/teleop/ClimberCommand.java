package com.team2502.robot2019.command.teleop;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ClimberCommand extends Command
{
    private boolean forwards;

    public ClimberCommand(boolean forwards)
    {
        requires(Robot.CLIMBER);
        this.forwards = forwards;
    }

    @Override
    protected void execute()
    {
        Robot.CLIMBER.climb(forwards);
    }

    @Override
    protected void end()
    {
        Robot.CLIMBER.stop();
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
