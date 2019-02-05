package com.team2502.robot2019.command.autonomous.ingredients;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class PrintCommand extends Command
{

    private String message;
    private int num_of_prints = 0;
    private int time_elapsed = 0;

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
        if (time_elapsed % 25 == 0)
        {
            System.out.println(message + " " + num_of_prints);
            num_of_prints++;
        }

        time_elapsed ++;

    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
