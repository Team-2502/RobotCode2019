package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.action.PeriodicAction;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class GoToTargetSimpleAction extends PeriodicAction
{
    private VisionWebsocket socket;
    private double totalSpeed;
    private ImmutableVector absLoc;

    public GoToTargetSimpleAction()
    {
        // TODO: add requires back with Resource
//        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void init() throws InterruptedException, ExecutionException, TimeoutException
    {
        DriverStation.reportWarning("Go to target initialized", false);
        socket = Robot.VISION_WEBSOCKET.get(5, TimeUnit.SECONDS);
        totalSpeed = 5;
    }

    @Override
    protected void execute()
    {
        VisionData visionData = socket.updateVisionData();
        if(visionData.isMeaningful())
        {
            absLoc = MathUtils.LinearAlgebra.relativeToAbsoluteCoord(visionData.getPos(), Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity(), Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading());
        }
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
    public void end()
    {
        Robot.DRIVE_TRAIN.driveSpeed(0);
        DriverStation.reportError("ended", false);
    }
}
