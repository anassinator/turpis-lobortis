import lejos.nxt.*;
import lejos.nxt.comm.*;
import lejos.util.*;

import java.io.DataInputStream;
import java.io.IOException;

/**
 * Controller for master NXT
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 1.2
 */
public class Turpis {
    // OBJECTS
    public static Robot robot;
    public static Odometer odometer;
    public static Display display;

    // FLAGS
    public static final boolean testing = true;

    // VARIABLES
    public static int[] courseInfo = {  1,      // STARTING CORNER
                                        2, -1,  // LOWER LEFT HOME ZONE
                                        4, 2,   // UPPER RIGHT HOME ZONE
                                        6, 8,   // LOWER LEFT OPPONENT ZONE
                                        8, 11,  // UPPER RIGHT OPPONENT ZONE
                                        1, 3,   // LOWER LEFT HOME TARGET
                                        8, 4,   // LOWER LEFT OPPONENT TARGET
                                        2,      // HOME FLAG
                                        3  };   // OPPONENT FLAG

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
        getBluetoothData();

        Navigation nav = new Navigation(robot, odometer, courseInfo);
        Localizer localizer = new Localizer(robot, odometer, nav, courseInfo[0]);

        // CLEAR DISPLAY
        LCD.clear();

        // START ODOMETER AND DISPLAY
        odometer.start();
        display.start();

        // LOCALIZE
        localizer.localize();

        // SET FLAGS
        robot.avoidPlz = true;

        // Correction corrector = new Correction(robot, odometer);
        // corrector.start();

        // SEARCH AND DESTROY
        nav.run();
    }

    /**
     * Waits until it receives a bluetooth message holding the course information
     * and then updates the array.
     * <p>
     * This method does not return immediately and may be slow.
     */
    public static void getBluetoothData() {
        NXTConnection conn = Bluetooth.waitForConnection();
        DataInputStream dis = conn.openDataInputStream();

        LCD.drawString("Opened", 0, 1);

        try {
            // WAIT
            while (dis.available() <= 0)
                Delay.msDelay(10);

            // PARSE
            int role = dis.readInt();
            dis.readChar();
            int startingCorner = dis.readInt();
            dis.readChar();
            int greenZoneLL_X = dis.readInt();
            dis.readChar();
            int greenZoneLL_Y = dis.readInt();
            dis.readChar();
            int greenZoneUR_X = dis.readInt();
            dis.readChar();
            int greenZoneUR_Y = dis.readInt();
            dis.readChar();
            int redZoneLL_X = dis.readInt();
            dis.readChar();
            int redZoneLL_Y = dis.readInt();
            dis.readChar();
            int redZoneUR_X = dis.readInt();
            dis.readChar();
            int redZoneUR_Y = dis.readInt();
            dis.readChar();
            int greenDZone_X = dis.readInt();
            dis.readChar();
            int greenDZone_Y = dis.readInt();
            dis.readChar();
            int redDZone_X = dis.readInt();
            dis.readChar();
            int redDZone_Y = dis.readInt();
            dis.readChar();
            int greenFlag = dis.readInt();
            dis.readChar();
            int redFlag = dis.readInt();
            dis.readChar();

            // FORMAT IT CORRECTLY
            if (role == 1)
                courseInfo = new int[] { startingCorner,
                                         greenZoneLL_X, greenZoneLL_Y,
                                         greenZoneUR_X, greenZoneUR_Y,
                                         redZoneLL_X, redZoneLL_Y,
                                         redZoneUR_X, redZoneUR_Y,
                                         greenDZone_X, greenDZone_Y,
                                         redDZone_X, redDZone_Y,
                                         greenFlag,
                                         redFlag };
            else
                courseInfo = new int[] { startingCorner,
                                         redZoneLL_X, redZoneLL_Y,
                                         redZoneUR_X, redZoneUR_Y,
                                         greenZoneLL_X, greenZoneLL_Y,
                                         greenZoneUR_X, greenZoneUR_Y,
                                         redDZone_X, redDZone_Y,
                                         greenDZone_X, greenDZone_Y,
                                         redFlag,
                                         greenFlag };
        } catch (Exception e) {}

        LCD.drawString("Done", 0, 2);

        // CLOSE SAFELY
        try {
            dis.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        conn.close();

        LCD.clear();
    }
}