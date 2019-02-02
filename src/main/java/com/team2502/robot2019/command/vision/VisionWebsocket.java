package com.team2502.robot2019.command.vision;

import com.github.ezauton.core.trajectory.geometry.ImmutableVector;

import java.io.*;
import java.net.Socket;

public class VisionWebsocket
{
    private final Socket socket;
    private final PrintWriter out;
    private final InputStreamReader reader;
    private final BufferedReader in;

    private VisionData visionData;

    public VisionWebsocket() throws IOException
    {this("team2502-tinker.local", 5800);}

    public VisionWebsocket(String host, int port) throws IOException
    {
        socket = new Socket(host,port);
        out = new PrintWriter(socket.getOutputStream(), true);
        reader = new InputStreamReader(socket.getInputStream());
        in = new BufferedReader(reader);
    }

    public VisionData updateVisionData()
    {
        // TODO: socket stuff...

        final String input;
        try {
            input = in.readLine();
        }catch(IOException e) {
            return visionData;
        }

        if(input == null)
            return visionData;

        final String[] args = input.split(",");
        if(args.length == 3) {
            try {
                final Double x = Double.parseDouble(args[0]);
                final Double y = Double.parseDouble(args[1]);
                final Double angle = Double.parseDouble(args[2]);

                visionData = new VisionData(x,y,angle);
            }catch(NumberFormatException ignored) { }
        }
        return visionData;
    }

    public ImmutableVector getPos()
    {
        return updateVisionData().getPos();
    }

    public double getAngle()
    {
        return updateVisionData().getAngle();
    }

    public void shutdown() throws IOException
    {
        reader.close();
        in.close();
        out.close();
        socket.close();
    }
}