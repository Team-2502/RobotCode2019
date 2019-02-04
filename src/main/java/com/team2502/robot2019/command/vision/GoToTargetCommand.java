package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.pathplanning.IPathSegment;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.QuinticSpline;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.command.Command;

import java.io.IOException;

public class GoToTargetCommand extends Command
{
    private VisionData lastTargetLocEstimation;

    private ImmutableVector lastAbsoluteTargetPos;
    private double lastAbsoluteTargetAngle;

    private QuinticSpline lastSpline;
    private final VisionWebsocket socket;
    private boolean stop;
    private double speed;

    public GoToTargetCommand() throws IOException
    {
        socket = new VisionWebsocket();
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
        speed = Math.max(Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity().mag(), 1);
    }

    @Override
    protected void execute()
    {
        VisionData newVisionData = socket.updateVisionData();
        if(newVisionData.isMeaningful())
        {
            lastTargetLocEstimation = newVisionData;

            double botheading = Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading();

            lastAbsoluteTargetPos = MathUtils.LinearAlgebra.rotate2D(lastTargetLocEstimation.getPos(), botheading).add(Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation());
            lastAbsoluteTargetAngle = botheading + lastTargetLocEstimation.getAngle();
        }
        if(lastTargetLocEstimation == null)
        {
            stop = true;
        }

        ImmutableVector loc = Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation();
        double angle = Robot.DRIVE_TRAIN.getLocEstimator().estimateHeading();


        Path path =  new SplinePPWaypoint.Builder()
                .add(loc.get(0), loc.get(1), angle, speed, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -Constants.Physical.DriveTrain.MAX_FPS2_ACCEL)
                .add(lastAbsoluteTargetPos.get(0), lastAbsoluteTargetPos.get(1), lastAbsoluteTargetAngle, speed, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -Constants.Physical.DriveTrain.MAX_FPS2_ACCEL)
                .buildPathGenerator().generate(0.05);

        IPathSegment next = path.getNext();

        Robot.DRIVE_TRAIN.driveTowardTransLoc(next.getSpeed(0.5), next.getTo());
    }

    @Override
    protected boolean isFinished()
    {
        return stop || Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation().dist(lastAbsoluteTargetPos) <= 1; // within a foot
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
    }
}
