package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

import java.io.IOException;

public class GoToTargetSimpleCommand extends Command
{
    private VisionWebsocket socket;
    private double totalSpeed;
    private ImmutableVector absLoc;

    public GoToTargetSimpleCommand()
    {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
        DriverStation.reportWarning("Go to target initialized", false);
        try
        {
            socket = new VisionWebsocket();
        }
        catch(IOException e)
        {
            DriverStation.reportError("Failed to create socket (whoops 3 potential precursor), socket port = " + Constants.Autonomous.PORT + ", mdns = " + Constants.Autonomous.COPROCESSOR_MDNS_ADDR, e.getStackTrace());
            super.cancel();
        }
        totalSpeed = 5;       // Math.max(Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity().mag(), 7);
    }

    @Override
    protected void execute()
    {
        VisionData visionData = socket.updateVisionData();
        if(visionData.isMeaningful())
        {
            absLoc =  MathUtils.LinearAlgebra.relativeToAbsoluteCoord(visionData.getPos(), Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity(), Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading());
        }
//        else {
//            Robot.DRIVE_TRAIN.driveSpeed(totalSpeed / 3D);
//        }
        if(absLoc != null)
        {
            Robot.DRIVE_TRAIN.driveTowardTransLoc(totalSpeed, absLoc);
        }
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }

    @Override
    protected void end()
    {
        try
        {
            socket.shutdown();
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }
        Robot.DRIVE_TRAIN.driveSpeed(0);

        DriverStation.reportError("ended", false);
    }
}
