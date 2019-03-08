package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.command.autonomous.ingredients.PointDriveCommand;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.io.IOException;

public class GoToTargetStupidCommand extends Command
{
    private VisionWebsocket socket;
    private double totalSpeed;

    public GoToTargetStupidCommand()
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
            if(visionData.getPos().get(0) <= (1/12) && visionData.getPos().get(0) >= (-1/12))
            {
                // Go Straight
                ImmutableVector absLoc = MathUtils.LinearAlgebra.relativeToAbsoluteCoord(visionData.getPos(), Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity(), Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading());
                Robot.DRIVE_TRAIN.driveSpeed((5+3D)/2D);
                System.out.println("centered");
            }
            else if(visionData.getPos().get(0) > 0)
            {
                // Turn Right
                Robot.DRIVE_TRAIN.runMotorsVelocity(5, 1);
                System.out.println("left");
            }
            else if (visionData.getPos().get(0) < 0)
            {
                // Turn Left
                Robot.DRIVE_TRAIN.runMotorsVelocity(1, 5);
                System.out.println("right");
            }
            else {
                //Imaginary
                Robot.DRIVE_TRAIN.runMotorsVelocity(5, 5);
                System.out.println("imaginary");
            }

        }
        else {
            Robot.DRIVE_TRAIN.driveSpeed(5D);
            System.out.println("not meaningful");
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
