package com.team2502.robot2019.command.autonomous.ingredients;

import com.github.ezauton.core.action.PeriodicAction;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;

import java.util.concurrent.TimeUnit;

public class MotionMagicDriveAction extends PeriodicAction
{
    private final double leftVel;
    private final double rightVel;
    private final double leftPos;
    private final double rightPos;


    /**
     * @param leftVel
     * @param rightVel
     *
     */
    public MotionMagicDriveAction(double leftPos, double rightPos, double leftVel, double rightVel)
    {this.leftPos = leftPos;
        this.rightPos = rightPos;
        this.leftVel = leftVel;
        this.rightVel = rightVel;
    }


    @Override
    protected void init()
    {
        try
        {
            Robot.DRIVE_TRAIN.take();
        }
        catch(InterruptedException e)
        {
            e.printStackTrace();
        }
        Robot.DRIVE_TRAIN.applyAutonomousPID();
        Robot.DRIVE_TRAIN.resetEncoderPosition();
    }

    @Override
    public void execute()
    {
        Robot.DRIVE_TRAIN.motionMagic(leftPos, rightPos, leftVel, rightVel);
    }

    @Override
    protected boolean isFinished() throws Exception
    {
        return false;
        //double leftFt = Robot.DRIVE_TRAIN.getFrontLeft().getSelectedSensorPosition() * Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS;
        //        double rightFt = Robot.DRIVE_TRAIN.getFrontRight().getSelectedSensorPosition() * Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS;
        //        return Math.abs(leftFt - leftPos) < 0.1 && Math.abs(rightFt - rightPos) < 0.1 && Math.abs(Robot.DRIVE_TRAIN.getFrontLeft().getSelectedSensorVelocity()) < 10  && Math.abs(Robot.DRIVE_TRAIN.getFrontRight().getSelectedSensorVelocity()) < 10;
    }

    @Override
    public void end() throws Exception
    {
        Robot.DRIVE_TRAIN.giveBack();
    }
}
