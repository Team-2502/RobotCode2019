package com.team2502.robot2019.command.vision;

import edu.wpi.first.wpilibj.command.Command;

import java.io.IOException;


public class AlwaysListeningCommand extends Command {

    VisionWebsocket socket = null;

    @Override
    protected void initialize() {
        super.initialize();


        try {
            socket = new VisionWebsocket("team2502-tinker.local", 5800);
        } catch (IOException e) {
            System.out.println("Websocket no connect");
        }

    }

    @Override
    protected void execute()
    {
        VisionData data = socket.updateVisionData();
        System.out.println(data);
    }

    @Override
    protected boolean isFinished(){ return false; }
}
