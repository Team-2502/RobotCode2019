package com.team2502.robot2019.command.teleop.climber;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ClimbCommand extends Command
{
    public enum Side
    {
        RIGHT,
        LEFT,
        BOTH
    }

    private boolean forwards;
    private Side side;

    public ClimbCommand(boolean forwards, Side side)
    {
        requires(Robot.CLIMBER);
        this.forwards = forwards;
        this.side = side;
    }

    @Override
    protected void execute()
    {
        switch(side)
        {
            case LEFT:
                Robot.CLIMBER.onlyLeftClimb(forwards);
                break;
            case RIGHT:
                Robot.CLIMBER.onlyRightClimb(forwards);
                break;
            case BOTH:
                Robot.CLIMBER.climb(forwards);
                break;
            default:
                Robot.CLIMBER.climb(forwards);
                break;
        }
    }

    @Override
    protected void end()
    {
        Robot.CLIMBER.stopClimb();
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
