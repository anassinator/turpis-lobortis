import lejos.nxt.*;
import lejos.nxt.comm.*;
import lejos.util.*;

import java.io.DataInputStream;
import java.io.IOException;

/**
 * Controller for master NXT
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 1.3
 */
public class Turpis {
    // OBJECTS
    public static Robot robot;
    public static Odometer odometer;
    public static Display display;

    // FLAGS
    public static final boolean testing = false;

    // VARIABLES
    public static int[] courseInfo = {  1,      // STARTING CORNER
                                        2, -1,  // LOWER LEFT HOME ZONE
                                        4, 2,   // UPPER RIGHT HOME ZONE
                                        2, 2,   // LOWER LEFT OPPONENT ZONE
                                        8, 11,  // UPPER RIGHT OPPONENT ZONE
                                        1, 3,   // LOWER LEFT HOME TARGET
                                        8, 4,   // LOWER LEFT OPPONENT TARGET
                                        2,      // HOME FLAG
                                        3  };   // OPPONENT FLAG */

    // MAIN
    public static void main(String[] args) {
        // SET VOLUME TO MAX
        Sound.setVolume(Sound.VOL_MAX);

        // WAIT UNTIL READY
        LCD.drawString("WELCOME", 5, 4);
        Button.waitForAnyPress();

        // SET UP ROBOT
        robot = new Robot();
        odometer = new Odometer(robot);
        display = new Display(odometer, robot, testing);

        // WAIT FOR BLUETOOTH
        courseInfo = getBluetoothData();

        Navigation nav = new Navigation(robot, odometer, courseInfo);
        Localizer localizer = new Localizer(robot, odometer, nav, courseInfo[0]);

        // CLEAR DISPLAY
        LCD.clear();

        // START ODOMETER AND DISPLAY
        odometer.start();
        display.start();

        if (!testing) {
            // LOCALIZE
            localizer.localize();

            // SET FLAGS
            robot.avoidPlz = true;

            // Correction corrector = new Correction(robot, odometer);
            // corrector.start();

            // SEARCH AND DESTROY
            nav.run();
        }
    }

    /**
     * Waits until it receives a bluetooth message holding the course information
     * and then updates the array.
     * <p>
     * This method does not return immediately and may be slow.
     */
    public static int[] getBluetoothData() {
        BluetoothConnection conn = new BluetoothConnection();
        Transmission t = conn.getTransmission();

        if (t == null) {
            LCD.drawString("Failed to read transmission", 0, 5);
            return courseInfo;
        } else {
            // GET DATA
            int role = t.role.getId();
            int corner = t.startingCorner.getId();
            int greenZoneLL_X = t.greenZoneLL_X;
            int greenZoneLL_Y = t.greenZoneLL_Y;
            int greenZoneUR_X = t.greenZoneUR_X;
            int greenZoneUR_Y = t.greenZoneUR_Y;
            int redZoneLL_X = t.redZoneLL_X;
            int redZoneLL_Y = t.redZoneLL_Y;
            int redZoneUR_X = t.redZoneUR_X;
            int redZoneUR_Y = t.redZoneUR_Y;
            int greenDZone_X = t.greenDZone_X;
            int greenDZone_Y = t.greenDZone_Y;
            int redDZone_X = t.redDZone_X;
            int redDZone_Y = t.redDZone_Y;
            int greenFlag = t.greenFlag;
            int redFlag = t.redFlag;

            // PRINT OUT THE TRANSMISSION INFORMATION
            if (testing) {
                LCD.clear();
                conn.printTransmission();
                Button.ENTER.waitForPress();
            }

            // FORMAT IT CORRECTLY
            if (role == 1)
                return new int[] { corner,
                                   greenZoneLL_X, greenZoneLL_Y,
                                   greenZoneUR_X, greenZoneUR_Y,
                                   redZoneLL_X, redZoneLL_Y,
                                   redZoneUR_X, redZoneUR_Y,
                                   greenDZone_X, greenDZone_Y,
                                   redDZone_X, redDZone_Y,
                                   greenFlag,
                                   redFlag };
            else
                return new int[] { corner,
                                   redZoneLL_X, redZoneLL_Y,
                                   redZoneUR_X, redZoneUR_Y,
                                   greenZoneLL_X, greenZoneLL_Y,
                                   greenZoneUR_X, greenZoneUR_Y,
                                   redDZone_X, redDZone_Y,
                                   greenDZone_X, greenDZone_Y,
                                   redFlag,
                                   greenFlag };
        }
    }
}