package com.team2502.robot2019.command.vision;

import com.team2502.robot2019.subsystem.vision.VisionWebsocket;

import java.io.IOException;
import java.net.Socket;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

public class VisionSocketCreator implements Callable<VisionWebsocket>
{

    private final long refreshDuration;
    private final TimeUnit timeUnit;
    private final String host;
    private final int port;

    public VisionSocketCreator(String host, int port, long refreshDuration, TimeUnit timeUnit)
    {
        this.host = host;
        this.port = port;
        this.refreshDuration = refreshDuration;
        this.timeUnit = timeUnit;
    }

    @Override
    public VisionWebsocket call() throws InterruptedException, IOException
    {
        Socket socket = null;
        boolean toReturn;
        do
        {
            try
            {
                socket = new Socket(host, port);
            }
            catch(IOException ignored) {}
            timeUnit.sleep(refreshDuration);
            toReturn = socket == null || !socket.isConnected();
            if(!toReturn) { timeUnit.sleep(refreshDuration); }
        }
        while(toReturn);

        return new VisionWebsocket(socket);
    }
}
