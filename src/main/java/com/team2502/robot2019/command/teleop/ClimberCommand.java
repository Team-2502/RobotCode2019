package com.team2502.robot2019.command.teleop;

import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.ClimberSubsystem;
import edu.wpi.first.wpilibj.command.Command;

public class ClimberCommand extends Command
{
    private boolean forwards;

    private ClimberSubsystem.ClimberSide side;

    public ClimberCommand(ClimberSubsystem.ClimberSide sides, boolean forwards)
    {
        requires(Robot.CLIMBER);
        this.forwards = forwards;
        this.side = sides;
    }

    @Override
    protected void execute()
    {
        Robot.CLIMBER.climb(side, forwards);
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
