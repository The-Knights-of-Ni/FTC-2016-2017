package org.ftcteam5206.subsystems;
/**
 * Created by Cat on 10/30/2016.
 * Purpose: takes in distance from launcher to goal, outputs appropriate hood angle and RPM based on data set
 * Assumptions: data file has equal intervals between each successive distance, has only numerical values in the file
 */

import java.io.*;
import java.util.*;

/**
 * Created by Cat on 10/30/2016.
 * Purpose: takes in distance from launcher to goal, outputs appropriate hood angle and RPM based on data set
 * Assumptions: data file has equal intervals between each successive distance, has only numerical values in the file
 */

import java.io.*;
import java.util.*;

public class LauncherLookup {
    private static HashMap data;

    // Reads data from file (of sorted distances) into a HashMap
    // stores a HashMap<Double, double[]> in the global variable data where the keys are distances and values are arrays of angles & RPM
    static {
        // Format: distance, hood angle, RPM
        String csvFile = "launchervalues.txt"; // Change file name/path
        BufferedReader reader;
        String line = "";
        String splitBy = " ";

        // Create a HashMap: distance --> {hood angle, RPM}
        HashMap<Double, double[]> map = new HashMap<Double, double[]>();

        try {
            reader = new BufferedReader(new FileReader(csvFile));
            while ((line = reader.readLine()) != null) {
                String[] values = line.split(splitBy);
                double[] angleRPM = new double[2];
                angleRPM[0] = Double.parseDouble(values[1]); // angle
                angleRPM[1] = Double.parseDouble(values[2]); // RPM
                map.put(Double.parseDouble(values[0]), angleRPM);
            }
        } catch (Exception e) {
            // e.printStackTrace();
        }

        data = map;
    }

    /* @param distance, a double that indicates the amount of distance from the turet to goal
     * @return hood angle & RPM in a double[] array     */
    public double[] outputAngleRPM(double distance) {
        double[] output = new double[2]; // Outputs [angle, RPM]

        HashMap<Double, double[]> map = data;

        Set set = map.entrySet();
        Iterator i = set.iterator();
        Map.Entry currentEntry;
        Map.Entry nextEntry = (Map.Entry)i.next();
        boolean flag = true; // flag is true when distance is not bounded by dist1 and dist2
        while(i.hasNext() && flag) {
            currentEntry = nextEntry;
            nextEntry = (Map.Entry)i.next();
            double dist1 = (double)(currentEntry.getKey());
            double dist2 = (double)(nextEntry.getKey());
            if (dist1 <= distance && distance <= dist2) {
                double angle1 = map.get(dist1)[0];
                double angle2 = map.get(dist2)[0];
                double rpm1 = map.get(dist1)[1];
                double rpm2 = map.get(dist2)[1];

                // Method 1: Output = average of the angles or RPM
                // output[0] = (angle1+angle2) / 2.0;
                // output[1] = (rpm1 + rpm2) / 2.0;

                // Method 2: Output = weighted average of the angles or RPM
                // based on the differences in dist1/dist2 to distance
                /* Example:
                Distance = 14, interval = 10
                Dist1 = 10, Angle1 = 30, rpm1 = 100
                Dist2 = 20, Angle2 = 40, rpm2 = 200
                Outputted angle = (14-10)/10 * 30 + (20-14)/10 * 40 = 36
                Outputted rpm = (14-10)/10 * 100 + (20-14)/10 * 200 = 160 */
                double interval = 10.0; // difference between distance values in the data set
                double diff1 = (distance - dist1)/interval;
                double diff2 = (dist2 - distance)/interval;
                output[0] = diff1 * angle1 + diff2 * angle2;
                output[1] = diff1 * rpm1 + diff2 * rpm2;

                flag = false;
            }

        }
        return output;
    }

//    // For debugging purposes
//    public static void main(String[] args) {
//      System.out.println(Arrays.toString(outputAngleRPM(14)));
//    }

}



