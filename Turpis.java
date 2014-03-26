import lejos.nxt.*;
import lejos.util.*;

/**
 * Controller for master NXT
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 1.1
 */
public class Turpis {
    // OBJECTS
    public static Robot robot;
    public static Odometer odometer;
    public static Display display;

    // FLAGS
    public static final boolean testing = false;

    // VARIABLES
    public static int[] courseInfo = {  4,      // STARTING CORNER
                                        2, -1,  // LOWER LEFT HOME ZONE
                                        4, 2,   // UPPER RIGHT HOME ZONE
                                        6, 8,   // LOWER LEFT OPPONENT ZONE
                                        8, 11,  // UPPER RIGHT OPPONENT ZONE
                                        1, 3,   // LOWER LEFT HOME TARGET
                                        8, 4,   // LOWER LEFT OPPONENT TARGET
                                        2,      // HOME FLAG
                                        3 };    // OPPONENT FLAG

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

        // courseInfo = getBluetoothData();

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
     * and then returns the 15 integers received into an array.
     * <p>
     * This method does not return immediately and may be slow.
     *
     * @return the course information
     */
    public static int[] getBluetoothData() {
        // ...
        return null;
    }
}