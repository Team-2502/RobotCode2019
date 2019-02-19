package com.team2502.robot2019.command.teleop.cargoactive;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

@Deprecated
public class CargoTopCommand extends Command
{
    private double speed;

    public CargoTopCommand(double speed)
    {
        this.speed = speed;
    }

    @Override
    protected void execute()
    {
        Robot.CARGO_ACTIVE.runTop(speed);
    }

    @Override
    protected void end()
    {
        Robot.CARGO_ACTIVE.runTop(0.0D);
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
