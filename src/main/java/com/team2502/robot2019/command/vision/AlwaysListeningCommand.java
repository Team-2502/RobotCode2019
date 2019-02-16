package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.Robot;
import com.team2502.robot2019.subsystem.vision.VisionData;
import com.team2502.robot2019.subsystem.vision.VisionWebsocket;
import edu.wpi.first.wpilibj.command.Command;

import java.io.IOException;


public class AlwaysListeningCommand extends Command {

    VisionWebsocket socket = null;
    VisionData lastData = null;

    public AlwaysListeningCommand() {
        setRunWhenDisabled(false);
    }

    @Override
    protected void initialize() {
        super.initialize();


        try {
            socket = new VisionWebsocket(Constants.Autonomous.COPROCESSOR_MDNS_ADDR, 5800);
        } catch (IOException e) {
            System.out.println("Websocket no connect");
        }

    }

    @Override
    protected void execute()
    {
        VisionData data = socket.updateVisionData();
        if(data.isMeaningful()) {
            lastData = data;
        }
//        System.out.println(data);

        double botheading = Robot.DRIVE_TRAIN.getRotEstimator().estimateHeading();

        ImmutableVector lastAbsoluteTargetPos = MathUtils.LinearAlgebra.rotate2D(lastData.getPos(), botheading).add(Robot.DRIVE_TRAIN.getLocEstimator().estimateLocation());
        double lastAbsoluteTargetAngle = botheading + lastData.getAngle();

        System.out.println("x: " + lastAbsoluteTargetPos.get(0) + ", y: " + lastAbsoluteTargetPos.get(1) + ", angle: " + lastAbsoluteTargetAngle);

    }

    @Override
    protected void end() {
        super.end();

        try {
            socket.shutdown();

        }
        catch (IOException e) {
            System.out.println("Shutdown failed");
        }
    }

    @Override
    protected boolean isFinished() { return false; }
}
