package com.team2502.robot2019.command.teleop;

import com.team2502.robot2019.OI;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCommand extends Command
{
    private boolean lastIteractive = false;
    public DriveCommand()
    {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
        Robot.DRIVE_TRAIN.applyTeleopPID();
        SmartDashboard.putNumber("deadband", 0.05);
    }

    @Override
    protected void execute()
    {
        if(!Robot.DRIVE_TRAIN.isTakenByAnyone())
        {
            double deadband = SmartDashboard.getNumber("deadband", 0.05);
            if(Math.abs(OI.JOYSTICK_DRIVE_LEFT.getY()) > deadband || Math.abs(OI.JOYSTICK_DRIVE_RIGHT.getY()) > deadband)
            {
                Robot.DRIVE_TRAIN.teleopDrive();
            }
            else
            {
                Robot.DRIVE_TRAIN.updateDifferentialDriveOftenEnough();
            }
            lastIteractive = true;
        }
        else {
            if(lastIteractive) {
                Robot.DRIVE_TRAIN.applyAutonomousPID();
                lastIteractive = false;
            }
        }
        double throttle = (-OI.JOYSTICK_DRIVE_LEFT.getThrottle() + 1)/2;
        Robot.DRIVE_TRAIN.setSpeedLimit(throttle);
        System.out.println("throttle = " + throttle);
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }

    @Override
    protected void end()
    {
        Robot.DRIVE_TRAIN.applyAutonomousPID();
    }
}
