package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.pathplanning.IPathSegment;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.QuinticSpline;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.pathplanning.purepursuit.SplinePPWaypoint;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.OI;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import com.team2502.robot2019.utils.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;

public class GoToTargetCommand extends Command
{
    private VisionData lastTargetLocEstimation;

    private ImmutableVector lastAbsoluteTargetPos;
    private double lastAbsoluteTargetAngle;

    private CircularBuffer angleBuffer = new CircularBuffer(6);
    private CircularBuffer xBuffer = new CircularBuffer(6);
    private CircularBuffer yBuffer = new CircularBuffer(6);

    private QuinticSpline lastSpline;
    private VisionWebsocket socket;
    private boolean stop = false;
    private double speed;

    public GoToTargetCommand()
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
//            stop = true;
        }
        speed = 6;       // Math.max(Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity().mag(), 7);
    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.update();
        VisionData newVisionData;
        try
        {
            newVisionData = socket.updateVisionData();
        }
        catch(NullPointerException e)
        {
            DriverStation.reportError("Null pointer in sockets (whoops 3)", e.getStackTrace());
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
//            System.out.println("newVisionData.isMeaningful() = " + newVisionData.isMeaningful());
//            System.out.println("lastAbsoluteTargetPos = " + lastAbsoluteTargetPos);
            lastAbsoluteTargetAngle = botheading + lastTargetLocEstimation.getAngle();
            angleBuffer.addValue(lastAbsoluteTargetAngle);
            xBuffer.addValue(lastAbsoluteTargetPos.get(0));
            yBuffer.addValue(lastAbsoluteTargetPos.get(1));
        }
        if(lastTargetLocEstimation == null)
        {
            stop = true;
        }

        ImmutableVector loc = Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation();
        double angle = Robot.DRIVE_TRAIN.getLocEstimator().estimateHeading();


//        Path path =

        PurePursuitMovementStrategy strategy = null;
        try
        {
            ImmutableVector knotVecRobot = MathUtils.LinearAlgebra.rotate2D(new ImmutableVector(0, 1), angle).mul(SmartDashboard.getNumber("knotVec0Mult", 1));
            ImmutableVector knotVecTarget = MathUtils.LinearAlgebra.rotate2D(new ImmutableVector(0, 1), angleBuffer.getMedian()).mul(SmartDashboard.getNumber("knotVec1Mult", 3));

            SplinePPWaypoint.Builder splineBuilder = new SplinePPWaypoint.Builder()
                    .add(loc.get(0), loc.get(1), knotVecRobot.get(0), knotVecRobot.get(1), speed, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -1000000)
                    .add(xBuffer.getMedian(), yBuffer.getMedian(), knotVecTarget.get(0), knotVecTarget.get(1), 0, Constants.Physical.DriveTrain.MAX_FPS2_ACCEL, -1000000);
            QuinticSpline spline = splineBuilder.buildSplines().get(0);
            System.out.println("spline.getEquation() = " + spline.getEquation());
            System.out.println("lastAbsoluteTargetAngle = " + lastAbsoluteTargetAngle);
            System.out.println("angleBuffer = " + angleBuffer.getMedian());
            System.out.println("angleBufferAvg = " + angleBuffer.getAverage());
            System.out.println("lastAbsoluteTargetAngleUnrounded = " + (angle + lastTargetLocEstimation.getAngle()));
            System.out.println("angle = " + angle);
            Path path = null;
            try
            {
                path = splineBuilder.buildPathGenerator().generate(0.05);
            }
            catch(IllegalArgumentException e)
            {
                DriverStation.reportWarning("No segments in path", e.getStackTrace());
                super.cancel();
            }
            strategy = new PurePursuitMovementStrategy(path, 1 / 12D);
            strategy.update(Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation(), 2);
            IPathSegment current = path.getCurrent();
            ImmutableVector closestPoint = current.getClosestPoint(loc);
//            double absoluteDistanceUsed = current.getAbsoluteDistance(closestPoint);
//            double speedUsed = current.getSpeed(absoluteDistanceUsed);


            Robot.DRIVE_TRAIN.driveTowardTransLoc(speed, strategy.getGoalPoint());
        }
        catch(NullPointerException e)
        {
            DriverStation.reportError("Could not find target", e.getStackTrace());
            System.out.println("lastAbsoluteTargetPos = " + lastAbsoluteTargetPos);
            System.out.println("newVisionData = " + newVisionData);
            System.out.println("meaningful = " + newVisionData.isMeaningful());
            System.out.println("strategy = " + strategy);
            if(strategy != null) { System.out.println("strategpgp = " + strategy.getGoalPoint()); }
        }
    }

    @Override
    protected boolean isFinished()
    {
        return false; // || OI.JOYSTICK_FUNCTION.getTrigger();
//        return lastAbsoluteTargetPos != null &&  (stop || Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation().dist(lastAbsoluteTargetPos) <= 1); // within a foot
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

        System.out.println("###################### ENDED ##################################");
//        DriverStation.reportError("ended", false);
    }
}
