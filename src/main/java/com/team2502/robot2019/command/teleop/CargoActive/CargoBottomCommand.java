package com.team2502.robot2019.command.teleop.CargoActive;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class CargoBottomCommand extends Command
{
    private double speed;

    public CargoBottomCommand(double speed)
    {
        this.speed = speed;
    }

    @Override
    protected void execute()
    {
        Robot.CARGO_ACTIVE.runBottom(speed);
    }

    @Override
    protected void end()
    {
        Robot.CARGO_ACTIVE.runBottom(0.0D);
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
