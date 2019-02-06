package com.team2502.robot2019.command.autonomous.ingredients;

import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Uses the PigeonIMU
 * Turns the robot specified degrees off the heading when the command is initialized
 * targetChange should be a double between -360 & 360
 */

public class TurnByDegreesCommand extends Command
{
    private double targetChange, speed;
    private int turnOpp;
    private double initialAngle, targetAngle;

    public TurnByDegreesCommand(double targetChange, double speed)
    {
        this.targetChange = targetChange;
        this.speed = speed;
    }

    @Override
    protected void initialize()
    {
        initialAngle = Robot.PIGEON.getCompassHeading();
        if(targetChange >= 0){turnOpp = -1;}
        else{turnOpp = 1;}
        targetAngle = initialAngle + targetChange;
    }

    @Override
    protected void execute()
    {
        // Run motors while the current heading isn't within the max error of the target heading
        Robot.DRIVE_TRAIN.runMotorsVelocity(-turnOpp*speed, turnOpp*speed);
    }

    @Override
    protected void end(){Robot.DRIVE_TRAIN.runMotorsVelocity(0,0);}

    @Override
    protected boolean isFinished()
    {return isWithinErrorMargin();}

    private boolean isWithinErrorMargin()
    {
        if(targetAngle < Robot.PIGEON.getCompassHeading() % 360 - Constants.Autonomous.MAX_TURN_ERROR
                || targetAngle > Robot.PIGEON.getCompassHeading() % 360 + Constants.Autonomous.MAX_TURN_ERROR)
        {return false;}
        else{return true;}
    }
}
