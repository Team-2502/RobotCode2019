package com.team2502.robot2019.command;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class LambdaCommand extends InstantCommand
{

    private final Runnable lambda;

    public LambdaCommand(Runnable lambda) {
        this.lambda = lambda;
    }

    @Override
    protected void execute()
    {
        lambda.run();
    }
}
