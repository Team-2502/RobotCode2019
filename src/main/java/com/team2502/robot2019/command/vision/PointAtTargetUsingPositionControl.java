package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team2502.robot2019.command.vision.GoToTargetNetworkTables.gttsc_speed;

public class PointAtTargetUsingPositionControl extends Command
{
    /**
     * SPeed the robot should go at
     */
    private double totalSpeed = 3;

    private double distanceToDrive;

    /**
     * The maximum possible offset that can occur. This has been measured to be 3 feet.
     */
    private final double max_offset = 3;

    /**
     * Data class that contains current offset
     */
    private VisionData visionInfo;


    public PointAtTargetUsingPositionControl()
    {
        requires(Robot.DRIVE_TRAIN);

        visionInfo = new VisionData(0,0,0);

        updateVisionData();
    }

    private void updateVisionData()
    {
        double tvecs1 = Robot.tvecs1Entry.getDouble(-9001);
        double tvecs2 = Robot.tvecs2Entry.getDouble(-9001);
        visionInfo.pos = new ImmutableVector(tvecs1, tvecs2);
        visionInfo.angle = Robot.angleEntry.getDouble(-9001);
    }

    @Override
    protected void initialize()
    {
        updateVisionData();

        DriverStation.reportWarning("Go to target initialized", false);

        double theta = Math.atan(visionInfo.pos.get(0) / visionInfo.pos.get(1));
        distanceToDrive = theta * Constants.Physical.DriveTrain.LATERAL_WHEEL_DIST_FT/2;
        Robot.DRIVE_TRAIN.resetEncoderPosition();
    }

    @Override
    protected void execute()
    {
        Robot.DRIVE_TRAIN.runMotorsPosition(distanceToDrive, -distanceToDrive);
//        Robot.DRIVE_TRAIN.runMotorsPosition(5, 5);
    }

    @Override
    protected boolean isFinished()
    {
        return false;
//        return Robot.DRIVE_TRAIN.getFrontLeft().getClosedLoopError() <= 30;
//        return visionInfo.getPos().get(0) <= 0.002;
    }

    @Override
    protected void end()
    {
        Robot.DRIVE_TRAIN.driveSpeed(0);
        DriverStation.reportError("ended", false);
    }

}
