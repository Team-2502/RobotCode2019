package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.pathplanning.IPathSegment;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.LookaheadBounds;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.recorder.Recording;
import com.github.ezauton.recorder.base.PurePursuitRecorder;
import com.github.ezauton.recorder.base.RobotStateRecorder;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import com.team2502.robot2019.utils.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;

public class GoToTargetCommand2 extends Command
{

    private Path splinePath;
    private Recording recording;

    public GoToTargetCommand2()
    {
        requires(Robot.DRIVE_TRAIN);
    }

    private PurePursuitMovementStrategy ppms;
    private final LookaheadBounds lookahead = Constants.Autonomous.getLookaheadBounds(Robot.DRIVE_TRAIN);
    private final double speed = 6;

    @Override
    protected void initialize()
    {
        // Make socket
        VisionWebsocket socket = null;
        try
        {
            socket = new VisionWebsocket();
        }
        catch(IOException e)
        {
            DriverStation.reportError("Could not connect to RPI.", e.getStackTrace());
            super.cancel();
        }

        // Prepare buffers for averaging numbers
        CircularBuffer angleBuffer = new CircularBuffer(6);
        CircularBuffer xBuffer = new CircularBuffer(6);
        CircularBuffer yBuffer = new CircularBuffer(6);

        // Read 10 images from PI into buffer
        for(int i = 0; i < 10; i++) {
            Robot.DRIVE_TRAIN.update();
            VisionData lastTargetLocEstimation = socket.updateVisionData();

            if(lastTargetLocEstimation.isMeaningful())
            {

                double botheading = Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading();

                ImmutableVector lastAbsoluteTargetPos = MathUtils.LinearAlgebra.rotate2D(lastTargetLocEstimation.getPos(), botheading).add(Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation());
                double lastAbsoluteTargetAngle = botheading + lastTargetLocEstimation.getAngle();

                angleBuffer.addValue(lastAbsoluteTargetAngle);
                xBuffer.addValue(lastAbsoluteTargetPos.get(0));
                yBuffer.addValue(lastAbsoluteTargetPos.get(1));

                System.out.println("yay");
            }
            else {
                i--;
            }
        }

        // Create the spline
        ImmutableVector loc = Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation();
        double angle = Robot.DRIVE_TRAIN.getLocEstimator().estimateHeading();

        ImmutableVector knotVecRobot = MathUtils.LinearAlgebra.rotate2D(new ImmutableVector(0, 1), angle).mul(SmartDashboard.getNumber("knotVec0Mult", 1));
        ImmutableVector knotVecTarget = MathUtils.LinearAlgebra.rotate2D(new ImmutableVector(0, 1), angleBuffer.getMedian()).mul(SmartDashboard.getNumber("knotVec1Mult", 3));


        System.out.println("targetLoc = " + new ImmutableVector(xBuffer.getMedian(), yBuffer.getMedian()));

        SplinePPWaypoint.Builder builder = new SplinePPWaypoint.Builder()
                .add(loc.get(0), loc.get(1), knotVecRobot.get(0), knotVecRobot.get(1), speed, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -1000000)
                .add(xBuffer.getMedian(), yBuffer.getMedian(), knotVecTarget.get(0), knotVecTarget.get(1), 2, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -1000000);
        System.out.println("splineEq = " + builder.buildSplines().get(0).getEquation());
        splinePath = builder
                .buildPathGenerator().generate(0.05);

        this.ppms = new PurePursuitMovementStrategy(splinePath, 0.5 / 12D);

        try
        {
            socket.shutdown();
        }
        catch(IOException e)
        {
            DriverStation.reportError("error shutting down socket", e.getStackTrace());
        }
        catch(NullPointerException e) {
            DriverStation.reportError("Socket did not exist", e.getStackTrace());
        }

        recording = new Recording();
        recording.addSubRecording(new RobotStateRecorder(RealClock.CLOCK, Robot.DRIVE_TRAIN.getLocEstimator(), Robot.DRIVE_TRAIN.getRotEstimator(), Constants.Physical.DriveTrain.LATERAL_WHEEL_DIST_FT, Constants.Physical.DriveTrain.ROBOT_LENGTH_FT));
        recording.addSubRecording(new PurePursuitRecorder(RealClock.CLOCK, splinePath, ppms));
    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.update();
        recording.update();
        ImmutableVector loc = Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation();
        ppms.update(loc, 0.4);


        IPathSegment current = splinePath.getCurrent();
        ImmutableVector closestPoint = current.getClosestPoint(loc);

        double absoluteDistanceUsed = current.getAbsoluteDistance(closestPoint);
        double speedUsed = current.getSpeed(absoluteDistanceUsed);

        Robot.DRIVE_TRAIN.driveTowardTransLoc(speedUsed, ppms.getGoalPoint());

    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }

    @Override
    protected void end()
    {
        System.out.println("ENDED");
        try
        {
            recording.save("rec.json");
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }
    }
}
