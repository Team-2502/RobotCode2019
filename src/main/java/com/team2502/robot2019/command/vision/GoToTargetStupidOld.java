package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;

public class GoToTargetStupidOld extends Command
{

    private VisionData visionData = new VisionData(0, 0, 0);

    private void updateVisionData()
    {
        double tvecs1 = Robot.tvecs1Entry.getDouble(-9001);
        double tvecs2 = Robot.tvecs2Entry.getDouble(-9001);
        visionData.pos = new ImmutableVector(tvecs1, tvecs2);
        visionData.angle = Robot.angleEntry.getDouble(-9001);
    }

    private double totalSpeed;

    public GoToTargetStupidOld()
    {
        requires(Robot.DRIVE_TRAIN);
    }

    @Override
    protected void initialize()
    {
        DriverStation.reportWarning("Go to target initialized", false);
        totalSpeed = SmartDashboard.getNumber("gttsc_speed",3);       // Math.max(Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity().mag(), 7);
    }

    @Override
    protected void execute()
    {
        updateVisionData();
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
                Robot.DRIVE_TRAIN.runMotorsVelocity(totalSpeed, totalSpeed/5);
                System.out.println("left");
            }
            else if (visionData.getPos().get(0) < 0)
            {
                // Turn Left
                Robot.DRIVE_TRAIN.runMotorsVelocity(totalSpeed/5, totalSpeed);
                System.out.println("right");
            }
            else {
                //Imaginary
                Robot.DRIVE_TRAIN.runMotorsVelocity(totalSpeed/2, totalSpeed/2);
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
        Robot.DRIVE_TRAIN.driveSpeed(0);

        DriverStation.reportError("ended", false);
    }
}