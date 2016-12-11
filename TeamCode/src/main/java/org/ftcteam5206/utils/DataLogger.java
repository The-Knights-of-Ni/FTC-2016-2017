package org.ftcteam5206.utils;

/**
 * Created by Dev on 10/12/2016.
 */

import android.content.Context;

import java.io.*;

public class DataLogger {
    //TODO: Single logging function that writes to debug, optionally telemetry and txt file on phone.
    //TODO: See if this needs its own thread.

    public static void dataLog(String s, Object o, Context c){
        //throw new UnsupportedOperationException();
        //TODO: Add write logic.
        /*
        try {
            FileOutputStream fOut = c.openFileOutput("file name here", 1);
            OutputStreamWriter writer = new OutputStreamWriter(fOut);
            writer.write(s);
        } catch(Exception e){
            e.printStackTrace();
        }
        */
    }

}
