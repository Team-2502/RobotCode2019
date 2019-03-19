package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.action.PeriodicAction;
import com.google.common.util.concurrent.AtomicDouble;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/**
 * Works for the purpose of turning towards the vision target.
 */
public class GoToTargetStupidAction extends PeriodicAction
{
    /**
     * Name of the SmartDashboard item that lets you change speed
     */
    private final String gttsc_speed = "gttsc_speed";

    /**
     * Instance of vision socket
     */
    private VisionWebsocket socket;

    /**
     * SPeed the robot should go at
     */
    private double totalSpeed = 3;

    /**
     * Instance of the PID Controller
     * <br>
     * Manages how the robot turns based on the offset
     */
    private PIDController pidController;

    /**
     * The desired difference between the wheel speeds. The PIDController writes to this AtomicDouble, then we read from it and drive accordingly.
     */
    private final AtomicDouble desiredWheelDifferential = new AtomicDouble();

    /**
     * The maximum possible offset that can occur. This has been measured to be 3 feet.
     */
    private final double max_offset = 3;
    private boolean stop = false;

    public GoToTargetStupidAction()
    {

        // TODO: add requires
//        requires(Robot.DRIVE_TRAIN);

        pidController = new PIDController(1.5, 0, 0, new PIDSource()
        {
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
                double lastPidGet = Math.min(max_offset, Math.max(-max_offset, socket.getPos().get(0)));
                System.out.println("lastPidGet = " + lastPidGet);
                return lastPidGet;
            }
        }, desiredWheelDifferential::set);


        SmartDashboard.putData("gototargetstupidcommand", pidController);
        SmartDashboard.putNumber(gttsc_speed, totalSpeed);
    }

    @Override
    protected void init() throws InterruptedException, ExecutionException, TimeoutException
    {
        DriverStation.reportWarning("Go to target initialized", false);

        socket = Robot.VISION_WEBSOCKET.get(5, TimeUnit.SECONDS);

        totalSpeed = SmartDashboard.getNumber(gttsc_speed, totalSpeed);
        pidController.setSetpoint(0);

        pidController.setInputRange(-max_offset, max_offset);
        pidController.setOutputRange(-totalSpeed, totalSpeed);
        pidController.setAbsoluteTolerance(1 / 12D);

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
            SmartDashboard.putBoolean("seesTarget", true);
            double velRight = totalSpeed + desiredWheelDifferential.get() / 2;
            double velLeft = totalSpeed - desiredWheelDifferential.get() / 2;
            SmartDashboard.putNumber("velLeft", velLeft);
            SmartDashboard.putNumber("velRight", velRight);
            Robot.DRIVE_TRAIN.runMotorsVelocity(velLeft, velRight);
        }
        else
        {
            SmartDashboard.putBoolean("seesTarget", false);
            Robot.DRIVE_TRAIN.driveSpeed(totalSpeed / 2);
            System.out.println("not meaningful");
        }
    }

    @Override
    protected boolean isFinished() //TODO: return true sometimes
    {
        return stop;
    }

    @Override
    public void end()
    {
        pidController.disable();
        Robot.DRIVE_TRAIN.driveSpeed(0);
        DriverStation.reportError("ended", false);
    }
}
