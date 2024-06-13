package org.firstinspires.ftc.teamcode.utilities;

//timer class
public class NanoTimer {
    public long startTime;
    NanoTimer() {
        resetTimer();
    }
    public void resetTimer(){
        startTime = System.nanoTime();
    };
    public long getElapsedTime(){
        return System.nanoTime()-startTime;
    }
    
}
