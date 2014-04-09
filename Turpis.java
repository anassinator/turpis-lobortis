import lejos.nxt.*;
import lejos.nxt.comm.*;
import lejos.util.*;

import java.io.DataInputStream;
import java.io.IOException;

/**
 * Controller for master NXT
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 2.0
 */
public class Turpis {
    // CONSTANTS
    private static double SIZE_OF_TILE = 30.48;

    // OBJECTS
    public static Robot robot;
    public static Odometer odometer;
    public static Display display;

    // FLAGS
    public static boolean testing = false;

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

    // MAIN
    public static void main(String[] args) {
        // SET VOLUME TO MAX
        Sound.setVolume(Sound.VOL_MAX);

        // WAIT UNTIL READY
        LCD.drawString("WELCOME", 5, 3);
        LCD.drawString("TEST", 0, 7);
        LCD.drawString("RUN", 13, 7);
        boolean done = false;
        while (!done) {
            int button = Button.waitForAnyPress();
            if (button == Button.ID_RIGHT) {
                done = true;
            } else if (button == Button.ID_LEFT) {
                done = true;
                testing = true;
            }
        }


        // SET UP ROBOT
        robot = new Robot();
        odometer = new Odometer(robot);
        display = new Display(odometer, robot, testing);

        // SET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(true);
        robot.rightColor.setFloodlight(true);

        if (!testing) {
            // WAIT FOR BLUETOOTH
            LCD.drawString("BLUETOOTH?", 4, 3);
            LCD.drawString("NO", 0, 7);
            LCD.drawString("YES", 13, 7);
            done = false;
            boolean bluetooth = false;
            while (!done) {
                int button = Button.waitForAnyPress();
                if (button == Button.ID_RIGHT) {
                    done = true;
                    bluetooth = true;
                } else if (button == Button.ID_LEFT) {
                    done = true;
                }
            }

            // GET BLUETOOTH DATA
            if (bluetooth)
                courseInfo = getBluetoothData();
        }

        // FIX COORDINATES SYSTEM
        transform();

        // SET UP NAVIGATION AND LOCALIZATION
        Navigation nav = new Navigation(robot, odometer, courseInfo);
        Localizer localizer = new Localizer(robot, odometer, nav);
        nav.localizer = localizer;

        // CLEAR DISPLAY
        LCD.clear();

        // START ODOMETER AND DISPLAY
        odometer.start();
        display.start();

        if (!testing) {
            // LOCALIZE
            localizer.localize();

            // SEARCH AND DESTROY
            nav.run();
        }
    }

    /**
     * Transform coordinates into robot's reference frame
     */
    public static void transform() {
        int startingCorner = courseInfo[0];

        int[] lowerLeftHome = transformCoordinates(courseInfo[1], courseInfo[2], startingCorner);
        int[] upperRightHome = transformCoordinates(courseInfo[3], courseInfo[4], startingCorner);
        courseInfo[1] = lowerLeftHome[0];
        courseInfo[2] = lowerLeftHome[1];
        courseInfo[3] = upperRightHome[0];
        courseInfo[4] = upperRightHome[1];

        int[] lowerLeftEnemy = transformCoordinates(courseInfo[5], courseInfo[6], startingCorner);
        int[] upperRightEnemy = transformCoordinates(courseInfo[7], courseInfo[8], startingCorner);
        courseInfo[5] = lowerLeftEnemy[0];
        courseInfo[6] = lowerLeftEnemy[1];
        courseInfo[7] = upperRightEnemy[0];
        courseInfo[8] = upperRightEnemy[1];

        int[] homeTarget = transformCoordinates(courseInfo[9], courseInfo[10], startingCorner);
        courseInfo[9] = homeTarget[0];
        courseInfo[10] = homeTarget[1];

        int[] enemyTarget = transformCoordinates(courseInfo[11], courseInfo[12], startingCorner);
        courseInfo[11] = enemyTarget[0];
        courseInfo[12] = enemyTarget[1];

        switch (startingCorner) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                courseInfo[1] = lowerLeftHome[0];
                courseInfo[2] = upperRightHome[1];

                courseInfo[3] = upperRightHome[0];
                courseInfo[4] = lowerLeftHome[1];

                courseInfo[5] = lowerLeftEnemy[0];
                courseInfo[6] = upperRightEnemy[1];

                courseInfo[7] = upperRightEnemy[0];
                courseInfo[8] = lowerLeftEnemy[1];

                courseInfo[9] = homeTarget[0];
                courseInfo[10] = homeTarget[1] - 1;

                courseInfo[11] = enemyTarget[0];
                courseInfo[12] = enemyTarget[1] - 1;
                break;
            case 4:
                courseInfo[1] = upperRightHome[0];
                courseInfo[2] = lowerLeftHome[1];

                courseInfo[3] = lowerLeftHome[0];
                courseInfo[4] = upperRightHome[1];

                courseInfo[5] = upperRightEnemy[0];
                courseInfo[6] = lowerLeftEnemy[1];

                courseInfo[7] = lowerLeftEnemy[0];
                courseInfo[8] = upperRightEnemy[1];

                courseInfo[9] = homeTarget[0] - 1;
                courseInfo[10] = homeTarget[1];

                courseInfo[11] = enemyTarget[0] - 1;
                courseInfo[12] = enemyTarget[1];
                break;
        }
    }

    /**
     * Transform coordinates into robot's reference frame
     *
     * @param x             x coordinate
     * @param y             y coordinate
     *
     * @return the x and y coordinates in an array
     */
    public static int[] transformCoordinates(int x, int y, int corner) {
        int[] transformed = {0, 0};
        switch (corner) {
            case 1:
                transformed[0] = x;
                transformed[1] = y;
                break;
            case 2:
                transformed[0] = y;
                transformed[1] = 10 - x;
                break;
            case 3:
                transformed[0] = 10 - x;
                transformed[1] = 10 - y;
                break;
            case 4:
                transformed[0] = 10 - y;
                transformed[1] = x;
                break;
        }

        return transformed;
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