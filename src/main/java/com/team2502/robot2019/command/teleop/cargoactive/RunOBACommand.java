package com.team2502.robot2019.command.teleop.cargoactive;

// ###############
// TODO: Make it spin oba

//

import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class RunOBACommand extends Command
{

    private double speed;
    public RunOBACommand(double speed)
    {
        requires(Robot.OBA);
        this.speed = speed;
    }

    protected void execute() { Robot.OBA.runOBA(speed);}

    @Override
    protected boolean isFinished()
    { return false; }

    protected void end(){Robot.OBA.stopOBA();}

}
