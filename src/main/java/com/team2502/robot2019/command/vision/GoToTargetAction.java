package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.action.PeriodicAction;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.QuinticSpline;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import com.team2502.robot2019.utils.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class GoToTargetAction extends PeriodicAction
{
    private VisionData lastTargetLocEstimation;

    private ImmutableVector lastAbsoluteTargetPos;
    private double lastAbsoluteTargetAngle;

    private CircularBuffer angleBuffer = new CircularBuffer(6);

    private QuinticSpline lastSpline;
    private VisionWebsocket socket;
    private boolean stop = false;
    private double speed;

    public GoToTargetAction()
    {
        // TODO: add back in using Resource interface
//        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void init()
    {
        stop = false;
        DriverStation.reportWarning("Go to target initialized", false);
        try
        {
            socket = Robot.VISION_WEBSOCKET.get(5, TimeUnit.SECONDS);
        }
        catch(InterruptedException | ExecutionException | TimeoutException e)
        {
            e.printStackTrace();
        }
        speed = 5;       // Math.max(Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity().mag(), 7);
    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.update();
        VisionData newVisionData = socket.updateVisionData();

        if(newVisionData.isMeaningful())
        {
            lastTargetLocEstimation = newVisionData;

            double botHeading = Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading();

            lastAbsoluteTargetPos = MathUtils.LinearAlgebra.rotate2D(lastTargetLocEstimation.getPos(), botHeading).add(Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation());
            lastAbsoluteTargetAngle = botHeading + lastTargetLocEstimation.getAngle();
            angleBuffer.addValue(lastAbsoluteTargetAngle);
        }
        if(lastTargetLocEstimation == null)
        {
            stop = true;
        }

        ImmutableVector loc = Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation();
        double angle = Robot.DRIVE_TRAIN.getLocEstimator().estimateHeading();

        SplinePPWaypoint.Builder splineBuilder = new SplinePPWaypoint.Builder()
                .add(loc.get(0), loc.get(1), angle, speed, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -Constants.Physical.DriveTrain.MAX_FPS2_ACCEL)
                .add(lastAbsoluteTargetPos.get(0), lastAbsoluteTargetPos.get(1), angle, speed, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -Constants.Physical.DriveTrain.MAX_FPS2_ACCEL);

        System.out.println("lastAbsoluteTargetAngle = " + lastAbsoluteTargetAngle);
        System.out.println("angleBuffer = " + angleBuffer.getAverage());
        System.out.println("lastAbsoluteTargetAngleUnrounded = " + (angle + lastTargetLocEstimation.getAngle()));
        System.out.println("angle = " + angle);
        Path path = splineBuilder.buildPathGenerator().generate(0.05);
        PurePursuitMovementStrategy strategy = new PurePursuitMovementStrategy(path, 1 / 12D);
        strategy.update(Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation(), 2);
        Robot.DRIVE_TRAIN.driveTowardTransLoc(speed, strategy.getGoalPoint());
    }

    @Override
    protected boolean isFinished()
    {
        return stop;
//        return lastAbsoluteTargetPos != null &&  (stop || Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation().dist(lastAbsoluteTargetPos) <= 1); // within a foot
    }

    @Override
    public void end()
    {
        Robot.DRIVE_TRAIN.driveSpeed(0);
        DriverStation.reportError("ended", false);
    }
}
