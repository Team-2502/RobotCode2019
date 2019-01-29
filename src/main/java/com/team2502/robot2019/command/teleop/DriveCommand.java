package com.team2502.robot2019.command.teleop;

import com.team2502.robot2019.Constants;
import com.team2502.robot2019.OI;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class DriveCommand extends Command
{

    public DriveCommand() {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void execute()
    {
        double leftVolts = -OI.JOYSTICK_DRIVE_LEFT.getY();
        double rightVolts = -OI.JOYSTICK_DRIVE_RIGHT.getY();

        leftVolts = leftVolts * leftVolts * Math.signum(leftVolts);
        rightVolts = rightVolts * rightVolts * Math.signum(rightVolts);

        if(Math.abs(leftVolts) < Constants.Physical.DriveTrain.THRESHOLD) {
            leftVolts = 0;
        }
        if (Math.abs(rightVolts) < Constants.Physical.DriveTrain.THRESHOLD) {
            rightVolts = 0;
        }

        Robot.DRIVE_TRAIN.runMotorsVoltage(leftVolts, rightVolts);

    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
