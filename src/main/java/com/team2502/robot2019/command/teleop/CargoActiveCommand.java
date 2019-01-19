package com.team2502.robot2019.command.teleop;

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class CargoActiveCommand extends Command {

    private double speed;

    public CargoActiveCommand(double speed)
    {
        requires(Robot.CARGO_ACTIVE);
        this.speed = speed;
    }

    @Override
    protected void execute()
    {
        Robot.CARGO_ACTIVE.runIntake(speed);
    }

    @Override
    protected boolean isFinished() {return false;}

    @Override
    protected void end(){Robot.CARGO_ACTIVE.stopIntake();}
}
