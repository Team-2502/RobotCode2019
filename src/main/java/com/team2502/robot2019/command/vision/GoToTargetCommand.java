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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

import java.io.IOException;

public class GoToTargetCommand extends Command
{
    private VisionData lastTargetLocEstimation;

    private ImmutableVector lastAbsoluteTargetPos;
    private double lastAbsoluteTargetAngle;

    private QuinticSpline lastSpline;
    private VisionWebsocket socket;
    private boolean stop;
    private double speed;

    public GoToTargetCommand()
    {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
        try
        {
            socket = new VisionWebsocket();
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }
        speed = Math.max(Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity().mag(), 1);
    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.update();
        VisionData newVisionData;
        try
        {
            newVisionData = socket.updateVisionData();
        } catch (NullPointerException e) {
            try
            {
                socket = new VisionWebsocket();
            }
            catch(IOException e1)
            {
                e1.printStackTrace();
            }

            newVisionData = socket.updateVisionData();

        }
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


//        Path path =

        try
        {
            QuinticSpline quinticSpline = new SplinePPWaypoint.Builder()
                    .add(loc.get(0), loc.get(1), angle, speed, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -Constants.Physical.DriveTrain.MAX_FPS2_ACCEL)
                    .add(lastAbsoluteTargetPos.get(0), lastAbsoluteTargetPos.get(1), lastAbsoluteTargetAngle, speed, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -Constants.Physical.DriveTrain.MAX_FPS2_ACCEL)
                    .buildSplines().get(0);
//                .buildPathGenerator().generate(0.05);

//        IPathSegment next = path.getNext();

            ImmutableVector next = quinticSpline.get(0.1);

            Robot.DRIVE_TRAIN.driveTowardTransLoc(0.5, next);
        } catch(NullPointerException e) {
            DriverStation.reportError("whoops 2", false);
        }
    }

    @Override
    protected boolean isFinished()
    {
        return lastAbsoluteTargetPos != null &&  (stop || Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation().dist(lastAbsoluteTargetPos) <= 1); // within a foot
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
