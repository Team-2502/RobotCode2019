package com.team2502.robot2019.command.vision;

import com.google.common.util.concurrent.AtomicDouble;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;

public class GoToTargetStupidCommand extends Command
{
    private VisionWebsocket socket;
    // Math.max(Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity().mag(), 7);
    private double totalSpeed = 4;
    private PIDController pidController;
    private final AtomicDouble desiredWheelDifferential = new AtomicDouble();
    private double offset = 2.5;
    private final double max_offset = 3;
    private double lastPidGet;

    public GoToTargetStupidCommand()
    {
        requires(Robot.DRIVE_TRAIN);

        pidController = new PIDController(0.2, 0, 0, new PIDSource() {
            PIDSourceType sourceType = PIDSourceType.kDisplacement;
            @Override
            public void setPIDSourceType(PIDSourceType pidSource)
            {
                throw new RuntimeException("You can't do that! (attempted to setPIDSourceType on anon class)");
            }

            @Override
            public PIDSourceType getPIDSourceType()
            {
                return sourceType;
            }

            @Override
            public double pidGet()
            {

//                if(socket.updateVisionData().isMeaningful())
//                    lastPidGet = Math.min(max_offset, Math.max(-max_offset, socket.getPos().get(0)));
                return 0;


//                System.out.println("input = " + input);
            }
        }, desiredWheelDifferential::set);

        pidController.setInputRange(-max_offset, max_offset);
        pidController.setOutputRange(-totalSpeed, totalSpeed);
        SmartDashboard.putData("gototargetstupidcommand", pidController);
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

        pidController.setSetpoint(0);
        pidController.enable();
    }

    @Override
    protected void execute()
    {
        VisionData visionData = socket.updateVisionData();
        SmartDashboard.putNumber("desiredratio", desiredWheelDifferential.get());
        SmartDashboard.putNumber("socket", visionData.getPos().get(0));
        if(visionData.isMeaningful())
        {
            double velRight = totalSpeed - desiredWheelDifferential.get()/2;
            double velLeft = totalSpeed + desiredWheelDifferential.get()/2;
            SmartDashboard.putNumber("velLeft", velLeft);
            SmartDashboard.putNumber("velRight", velRight);
            Robot.DRIVE_TRAIN.runMotorsVelocity(velLeft, velRight);
//            if(visionData.getPos().get(0) <= (1/12) && visionData.getPos().get(0) >= (-1/12))
//            {
//                // Go Straight
//                ImmutableVector absLoc = MathUtils.LinearAlgebra.relativeToAbsoluteCoord(visionData.getPos(), Robot.DRIVE_TRAIN.getLocEstimator().estimateAbsoluteVelocity(), Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading());
//                Robot.DRIVE_TRAIN.driveSpeed((5+3D)/2D);
//                System.out.println("centered");
//            }
//            else if(visionData.getPos().get(0) > 0)
//            {
//                // Turn Right
//                Robot.DRIVE_TRAIN.runMotorsVelocity(5, 1);
//                System.out.println("left");
//            }
//            else if (visionData.getPos().get(0) < 0)
//            {
//                // Turn Left
//                Robot.DRIVE_TRAIN.runMotorsVelocity(1, 5);
//                System.out.println("right");
//            }
//            else {
//                //Imaginary
//                Robot.DRIVE_TRAIN.runMotorsVelocity(5, 5);
//                System.out.println("imaginary");
//            }

        }
        else {
            Robot.DRIVE_TRAIN.driveSpeed(totalSpeed);
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
