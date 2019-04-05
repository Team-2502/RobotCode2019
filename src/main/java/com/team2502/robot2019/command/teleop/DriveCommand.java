package com.team2502.robot2019.command.teleop;

import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.core.utils.Stopwatch;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.OI;
import com.team2502.robot2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

import java.util.concurrent.TimeUnit;

public class DriveCommand extends Command
{

    private Stopwatch stopwatch;

    private double lastLeftVelTarget = Double.NaN;
    private double lastRightVelTarget = Double.NaN;

    public DriveCommand() {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
        stopwatch = new Stopwatch(RealClock.CLOCK);
        stopwatch.init();
    }

    //    @Override
    protected void executeVolts()
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
    protected void execute() //executeVelocity
    {
        double leftVolts = -OI.JOYSTICK_DRIVE_LEFT.getY();
        double rightVolts = -OI.JOYSTICK_DRIVE_RIGHT.getY();

        // skipping input squaring

        double leftVelTarget = leftVolts * Constants.Physical.DriveTrain.MAX_FPS_SPEED;
        double rightVelTarget = rightVolts * Constants.Physical.DriveTrain.MAX_FPS_SPEED;

        if(!Double.isNaN(lastLeftVelTarget) && !Double.isNaN(lastRightVelTarget)) {
            double dt = stopwatch.pop(TimeUnit.SECONDS);


            leftVelTarget = handleMaxAcc(leftVelTarget, lastLeftVelTarget, dt);
            rightVelTarget = handleMaxAcc(rightVelTarget, lastRightVelTarget, dt);
        }

        Robot.DRIVE_TRAIN.runMotorsVelocity(leftVelTarget, rightVelTarget);

        lastLeftVelTarget = leftVelTarget;
        lastRightVelTarget = rightVelTarget;
    }

    private double handleMaxAcc(double desiredVelocity, double lastVelocity, double dt) {

        double currentAcc = Math.abs(desiredVelocity - lastVelocity) / dt;
        double ret;
        if(currentAcc < Constants.Physical.DriveTrain.MAX_FPS2_ACCEL) { // within bounds
            ret = desiredVelocity; // as is
        }
        else // out of bounds
        {
            System.out.println("ur driving 2 fast");
            double sign = Math.signum(desiredVelocity - lastVelocity); // limit
            ret = lastVelocity + sign * Constants.Physical.DriveTrain.MAX_FPS2_ACCEL * dt;
        }
        System.out.println(currentAcc);
        return ret;
    }
    @Override
    protected boolean isFinished()
    {
        return false;
    }
}
