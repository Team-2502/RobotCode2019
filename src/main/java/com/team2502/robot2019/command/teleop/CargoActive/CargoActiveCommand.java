package com.team2502.robot2019.command.teleop.CargoActive;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

@Deprecated
public class CargoActiveCommand extends Command {

    private final boolean top;
    private double speed;

    public CargoActiveCommand(double speed, boolean top)
    {
        this.top = top;
        requires(Robot.CARGO_ACTIVE);
        this.speed = speed;
    }

    @Override
    protected void execute()
    {
        if(top)
        {
            Robot.CARGO_ACTIVE.runTop(speed);
        }
        else {
            Robot.CARGO_ACTIVE.runBottom(speed);
        }
    }

    @Override
    protected boolean isFinished() {return false;}

    @Override
    protected void end(){Robot.CARGO_ACTIVE.stopIntake();}
}
