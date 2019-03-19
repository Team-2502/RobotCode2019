package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.command.Command;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/**
 * @deprecated
 */
public class AlwaysListeningCommand extends Command
{

    private VisionData lastData = null;
    private VisionWebsocket visionWebsocket;

    public AlwaysListeningCommand()
    {
        setRunWhenDisabled(false);
    }

    @Override
    protected void initialize()
    {
        super.initialize();
        try
        {
            visionWebsocket = Robot.VISION_WEBSOCKET.get(5, TimeUnit.SECONDS);
        }
        catch(InterruptedException | ExecutionException | TimeoutException e)
        {
            e.printStackTrace();
        }
    }

    @Override
    protected void execute()
    {
        VisionData data = visionWebsocket.updateVisionData();
        if(data.isMeaningful())
        {
            lastData = data;
        }

        double botHeading = Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading();

        ImmutableVector lastAbsoluteTargetPos = MathUtils.LinearAlgebra.rotate2D(lastData.getPos(), botHeading).add(Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation());
        double lastAbsoluteTargetAngle = botHeading + lastData.getAngle();

        System.out.println("x: " + lastAbsoluteTargetPos.get(0) + ", y: " + lastAbsoluteTargetPos.get(1) + ", angle: " + lastAbsoluteTargetAngle);

    }

    @Override
    protected boolean isFinished() { return false; }
}
