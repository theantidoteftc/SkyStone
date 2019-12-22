package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

//STEVEN'S CODE (written 11/15/19) goodfaithday
//NEVER EVER DELETE WITHOUT ADITYA'S PERMISSION
//OR LIKE NOT AT ALL, THIS IS GOD'S CODE

public class StateMachine {

    int currentState;

    long stateStartTime;

    boolean stateFinished = true;

    public StateMachine() {
    }


    public void nextState() {
        stateStartTime = SystemClock.uptimeMillis();
        stateFinished = true;
        currentState++;
    }

    public void setState(int state) {
        nextState();
        currentState = state;
    }

    public double timeSinceStart() {
        return (SystemClock.uptimeMillis() - stateStartTime)/1000.0;
    }

    public int getCurrentState() {
        return currentState;
    }

    //updates the finished variable to false
    public void update() {
        stateFinished = false;
    }

    public boolean isFirstUpdate() {
        return stateFinished;
    }

}
