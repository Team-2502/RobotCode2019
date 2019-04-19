package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.action.PeriodicAction;
import com.github.ezauton.core.action.TimedPeriodicAction;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.google.common.util.concurrent.AtomicDouble;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.utils.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;

public class GoToTargetNTAction extends PeriodicAction
{

    private static final TimeUnit defaultDurationUnit = TimeUnit.MILLISECONDS;
    private static final long defaultDuration = 20;
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

    /**
     * Data class that contains current offset
     */
    private VisionData visionInfo;

    /**
     * Circular buffer that keeps track of offsets in past loop iterations
     */
    private CircularBuffer errorBuffer;

    public GoToTargetNTAction()
    {
        super(defaultDuration, defaultDurationUnit);
        visionInfo = new VisionData(0,0,0);

        updateVisionData();

        pidController = new PIDController(Constants.Autonomous.visionkP, Constants.Autonomous.visionkI, Constants.Autonomous.visionkD, new PIDSource() {
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
                double lastPidGet = Math.min(max_offset, Math.max(-max_offset, visionInfo.getPos().get(0)));
                System.out.println("lastPidGet = " + lastPidGet);
                return lastPidGet;
            }
        }, desiredWheelDifferential::set);


        SmartDashboard.putData("gototargetstupidcommand", pidController);
        SmartDashboard.putNumber(GoToTargetNetworkTables.gttsc_speed, totalSpeed);

    }

    @Override
    protected void init() throws Exception
    {
        DriverStation.reportWarning("Go to target initialized", false);
        totalSpeed = SmartDashboard.getNumber(GoToTargetNetworkTables.gttsc_speed, totalSpeed);
        pidController.setSetpoint(0);

        pidController.setInputRange(-max_offset, max_offset);
        pidController.setOutputRange(-totalSpeed, totalSpeed);
        pidController.setAbsoluteTolerance(.01);
        pidController.enable();

        errorBuffer = new CircularBuffer(800);
        SmartDashboard.putNumber("pleaseUnstick", 0);

        Robot.DRIVE_TRAIN.getResource().take();
    }

    @Override
    protected void execute() throws Exception
    {
        updateVisionData();

        SmartDashboard.putNumber("desiredratio", desiredWheelDifferential.get());
        SmartDashboard.putNumber("socket", visionInfo.getPos().get(0));

        if(visionInfo.isMeaningful())
        {
            double velRight = totalSpeed + desiredWheelDifferential.get() / 2;
            double velLeft = totalSpeed - desiredWheelDifferential.get() / 2;
            SmartDashboard.putNumber("velLeft", velLeft);
            SmartDashboard.putNumber("velRight", velRight);
            Robot.DRIVE_TRAIN.runMotorsVelocity(velLeft, velRight);
            errorBuffer.addValue(visionInfo.pos.get(0));
        }
        else
        {
            Robot.DRIVE_TRAIN.runMotorsVoltage(0, 0);
            System.out.println("not meaningful");
        }
    }

    private void updateVisionData()
    {
        double tvecs1 = Robot.tvecs1Entry.getDouble(-9001) - Constants.Autonomous.visionOffset;
        double tvecs2 = Robot.tvecs2Entry.getDouble(-9001);
        visionInfo.pos = new ImmutableVector(tvecs1, tvecs2);
        visionInfo.angle = Robot.angleEntry.getDouble(-9001);
    }

    @Override
    protected boolean isFinished()
    {
        return Math.abs(visionInfo.getPos().get(1)) <= 0.7 || visionInfo.getPos().get(0) == -9001;
    }

    @Override
    public void end() throws Exception
    {
        Robot.DRIVE_TRAIN.getResource().giveBack();
        pidController.disable();
        Robot.DRIVE_TRAIN.runMotorsVoltage(0, 0);
        DriverStation.reportError("ended", false);
        SmartDashboard.putNumber("pleaseUnstick", 1);
    }
}
