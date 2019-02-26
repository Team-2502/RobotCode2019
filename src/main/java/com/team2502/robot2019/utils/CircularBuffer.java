package com.team2502.robot2019.utils;

import java.util.LinkedList;
import java.util.List;

/**
 * Implements a simple circular buffer.
 *
 * @author Cheesy Poofs, 2017
 */
public class CircularBuffer
{
    int mWindowSize;
    LinkedList<Double> mSamples;
    double mSum;

    public CircularBuffer(int window_size)
    {
        mWindowSize = window_size;
        mSamples = new LinkedList<Double>();
        mSum = 0.0;
    }

    public void clear()
    {
        mSamples.clear();
        mSum = 0.0;
    }

    public double getAverage()
    {
        if(mSamples.isEmpty())
        { return 0.0; }
        return mSum / mSamples.size();
    }

    public double getMedian() {
        List<Double> doubleList = (List<Double>) mSamples.clone();
        doubleList.sort(Double::compare);
        try
        {
            return doubleList.get(doubleList.size() / 2);
        } catch (IndexOutOfBoundsException e) {
            return 0;
        }
    }

    public void recomputeAverage()
    {
        // Reset any accumulation drift.
        mSum = 0.0;
        if(mSamples.isEmpty())
        { return; }
        for(Double val : mSamples)
        {
            mSum += val;
        }
        mSum /= mWindowSize;
    }

    public void addValue(double val)
    {
        mSamples.addLast(val);
        mSum += val;
        if(mSamples.size() > mWindowSize)
        {
            mSum -= mSamples.removeFirst();
        }
    }

    public int getNumValues()
    {
        return mSamples.size();
    }

    public boolean isFull()
    {
        return mWindowSize == mSamples.size();
    }
}
