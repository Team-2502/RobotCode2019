package com.team2502.robot2019.command.autonomous.ingredients;

import edu.wpi.first.wpilibj.command.Command;

public class PrintCommand extends Command
{

    private String message;

    public PrintCommand(String message)
    {
        this.message = message;

        /* TODO: This class would be used to test the AbortAutoCommand.
         * Make this command require a subsystem, such as DrivetrainSubsystem.
         * Schedule this command in Robot.java such that it is scheduled before DriveCommand.
         * Assign AbortAutoCommand to a button, and press the button.
         * If AbortAutoCommand works as intended, this command will stop hoarding the DrivetrainSubsystem, and teleop should resume.
         * Prints should also stop coming from this command.
         */

        //requires(Robot.SUBSYSTEM);
    }

    @Override
    protected void execute()
    {
        System.out.println(message);
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
