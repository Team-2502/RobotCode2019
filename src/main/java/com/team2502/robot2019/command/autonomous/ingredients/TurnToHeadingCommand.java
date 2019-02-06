package com.team2502.robot2019.command.autonomous.ingredients;

import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Uses the PigeonIMU
 * Turns the robot to a specified heading, No matter the heading when the command is initialized
 * targetAngle should be a double between 0 & 360
 */

public class TurnToHeadingCommand extends Command
{

    private double targetAngle, speed;
    private int turnOpp;

    public TurnToHeadingCommand(double targetAngle, double speed)
    {
        this.targetAngle = targetAngle;
        this.speed = speed;
    }

    @Override
    protected void initialize(){
        double initialAngle = Robot.PIGEON.getCompassHeading() % 360;

        // Calculate whether to go left or right
        if(Math.abs(targetAngle - (initialAngle)) > Math.abs(targetAngle - (initialAngle - 360)))
        {turnOpp = -1;} // Turn right
        else{turnOpp = 1;} // Turn left
    }

    @Override
    protected void execute()
    {
        // Run motors while the current heading isn't within the max error of the target heading
        Robot.DRIVE_TRAIN.runMotorsVelocity(-turnOpp * speed, turnOpp * speed);
    }

    @Override
    protected void end(){
        Robot.DRIVE_TRAIN.runMotorsVelocity(0,0);
    }

    @Override
    protected boolean isFinished(){return isWithinErrorMargin();}

    private boolean isWithinErrorMargin()
    {
        if(targetAngle > Robot.PIGEON.getCompassHeading() % 360 - Constants.Autonomous.MAX_TURN_ERROR
                || targetAngle < Robot.PIGEON.getCompassHeading() % 360 + Constants.Autonomous.MAX_TURN_ERROR)
        {return true;}
        else{return false;}
    }
}
