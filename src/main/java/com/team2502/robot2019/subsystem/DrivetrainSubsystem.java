package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.github.ezauton.core.action.require.BaseResource;
import com.github.ezauton.core.actuators.IVelocityMotor;
import com.github.ezauton.core.localization.IRotationalLocationEstimator;
import com.github.ezauton.core.localization.Updateable;
import com.github.ezauton.core.localization.UpdateableGroup;
import com.github.ezauton.core.localization.estimators.EncoderRotationEstimator;
import com.github.ezauton.core.localization.estimators.TankRobotEncoderEncoderEstimator;
import com.github.ezauton.core.localization.sensors.Encoders;
import com.github.ezauton.core.localization.sensors.ITranslationalDistanceSensor;
import com.github.ezauton.core.localization.sensors.IVelocityEstimator;
import com.github.ezauton.core.robot.implemented.TankRobotTransLocDriveable;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.github.ezauton.wpilib.motors.ITypicalMotor;
import com.github.ezauton.wpilib.motors.MotorControllers;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.RobotMap;
import com.team2502.robot2019.command.teleop.DriveCommand;
import com.team2502.robot2019.subsystem.interfaces.IDriveTrain;
import com.team2502.robot2019.utils.IPIDTunable;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DrivetrainSubsystem extends Subsystem implements IPIDTunable, IDriveTrain, Updateable
{
    private final WPI_TalonSRX backLeft;
    private final WPI_TalonSRX backRight;

    private final WPI_TalonSRX frontLeft;
    private final WPI_TalonSRX frontRight;

    private final ITypicalMotor left;
    private final ITypicalMotor right;

    private final TankRobotTransLocDriveable trtls;
    private final EncoderRotationEstimator locEst;
    private final IRotationalLocationEstimator rotEst;
    private final IVelocityEstimator velEst;


    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private boolean forward = true;

    private final BaseResource resource = new BaseResource();
    private final UpdateableGroup updateableGroup;
    private final ITranslationalDistanceSensor leftSensor;
    private final ITranslationalDistanceSensor rightSensor;

    public DrivetrainSubsystem()
    {
        backLeft = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_BACK_LEFT);
        backRight = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_BACK_RIGHT);
        frontLeft = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_FRONT_LEFT);
        frontRight = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_FRONT_RIGHT);

        frontLeft.setSelectedSensorPosition(0);
        frontLeft.setSensorPhase(true);
        frontRight.setSelectedSensorPosition(0);

        left = MotorControllers.fromSeveralCTRE(frontLeft, 0, backLeft);
        right = MotorControllers.fromSeveralCTRE(frontRight, 0, backRight);

        leftSensor = Encoders.toTranslationalDistanceSensor(
                Constants.Physical.DriveTrain.ENC_UNITS_TO_FEET,
                (long) Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS,
                left
                                                           );
        rightSensor = Encoders.toTranslationalDistanceSensor(
                Constants.Physical.DriveTrain.ENC_UNITS_TO_FEET,
                (long) Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS,
                right
                                                            );

        rotEst = () -> MathUtils.Kinematics.navXToRad(Robot.NAVX.getAngle());
        velEst = () -> (leftSensor.getVelocity() + rightSensor.getVelocity()) / 2;

        locEst = new EncoderRotationEstimator(rotEst, new ITranslationalDistanceSensor()
        {
            @Override
            public double getPosition()
            {
                return (leftSensor.getPosition() + rightSensor.getPosition()) / 2;
            }

            @Override
            public double getVelocity()
            {
                return (leftSensor.getVelocity() + rightSensor.getVelocity()) / 2;
            }
        }); //new TankRobotEncoderEncoderEstimator(leftSensor, rightSensor, Constants.Physical.DriveTrain.TANK_ROBOT_CONSTANTS);
        locEst.reset();


        trtls = new TankRobotTransLocDriveable(left, right, locEst, rotEst, Constants.Physical.DriveTrain.TANK_ROBOT_CONSTANTS);

        // DO NOT CHANGE
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        updateableGroup = new UpdateableGroup(locEst);
    }

    @Override
    protected void initDefaultCommand()
    {
        setDefaultCommand(new DriveCommand()); //TODO: DriveCommand
    }

    public void runMotors(ControlMode controlMode, double leftVal, double rightVal)
    {
        frontLeft.set(controlMode, leftVal);
        frontRight.set(controlMode, rightVal);
    }

    public void runMotorsVelocity(double leftVal, double rightVal)
    {
        runMotors(ControlMode.Velocity, leftVal * Constants.PER100MS_TO_SECONDS * Constants.Physical.Encoder.RAW_UNIT_PER_FT,
                rightVal * Constants.PER100MS_TO_SECONDS * Constants.Physical.Encoder.RAW_UNIT_PER_FT);
    }

    @Deprecated
    public void runMotorsVoltage(double leftVolts, double rightVolts)
    {
//        runMotors(ControlMode.PercentOutput, leftVolts, rightVolts);
//        left.runVoltage(forward ? leftVolts : -rightVolts);
//        right.runVoltage(forward ? rightVolts : -leftVolts);
        left.runVoltage(leftVolts);
        right.runVoltage(rightVolts);
    }

    @Override
    public IVelocityMotor getLeft()
    {
        return left;
    }

    @Override
    public IVelocityMotor getRight()
    {
        return right;
    }

    @Override
    public EncoderRotationEstimator getLocEstimator()
    {
        return locEst;
    }

    @Override
    public IRotationalLocationEstimator getRotEstimator()
    {
        return rotEst;
    }

    @Override
    public IVelocityEstimator getVelocityEstimator()
    {
        return velEst;
    }

    public WPI_TalonSRX getFrontLeft()
    {
        return frontLeft;
    }

    public WPI_TalonSRX getFrontRight()
    {
        return frontRight;
    }

    @Override
    public double getkP()
    {
        return kP;
    }

    @Override
    public void setkP(double kP)
    {
        this.kP = kP;
        applyPID();
    }

    @Override
    public double getkI()
    {
        return kI;
    }

    @Override
    public void setkI(double kI)
    {
        this.kI = kI;
        applyPID();
    }

    @Override
    public double getkD()
    {
        return kD;
    }

    @Override
    public void setkD(double kD)
    {
        this.kD = kD;
        applyPID();
    }

    @Override
    public double getkF()
    {
        return kF;
    }

    @Override
    public void setkF(double kF)
    {
        this.kF = kF;
        applyPID();
    }

    @Override
    public void setPID(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        applyPID();
    }

    public void applyPID()
    {
        frontLeft.config_kP(0, kP, Constants.INIT_TIMEOUT);
        frontLeft.config_kI(0, kI, Constants.INIT_TIMEOUT);
        frontLeft.config_kD(0, kD, Constants.INIT_TIMEOUT);
        frontLeft.config_kF(0, kF, Constants.INIT_TIMEOUT);

        frontRight.config_kP(0, kP, Constants.INIT_TIMEOUT);
        frontRight.config_kI(0, kI, Constants.INIT_TIMEOUT);
        frontRight.config_kD(0, kD, Constants.INIT_TIMEOUT);
        frontRight.config_kF(0, kF, Constants.INIT_TIMEOUT);
    }

    @Override
    public boolean driveTowardTransLoc(double speed, ImmutableVector loc)
    {
        System.out.println("driveTowardTransLoc()");
        return trtls.driveTowardTransLoc(speed, loc);
    }

    @Override
    public boolean driveSpeed(double speed)
    {
        return trtls.driveSpeed(speed);
    }


    @Override
    public BaseResource getResource()
    {
        return resource;
    }

    @Override
    public boolean update()
    {
        boolean ret = updateableGroup.update();
//        System.out.println(locEst.estimateLocation());
//        System.out.println("locEst.estimateLocation() = " + locEst.estimateLocation());
//        System.out.println("locEst.estimateHeading() = " + locEst.estimateHeading());
        return ret;
    }

    public boolean isForward()
    {
        return forward;
    }

    public void setForward(boolean forward)
    {
        this.forward = forward;
    }
}
