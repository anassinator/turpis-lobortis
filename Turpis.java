/**
 * Turpis.java
 * Controller for master NXT
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.1
 */

import lejos.nxt.*;

public class Turpis {
    public static int[] courseInfo = { 1 };

    public static Robot robot = new Robot();
    public static Odometer odometer = new Odometer(robot);
    public static Map map = new Map(robot, odometer);
    public static Display display = new Display(odometer);

    public static void main(String[] args) {
        // SET VOLUME TO MAX
        Sound.setVolume(100);

        LCD.drawString("WELCOME", 5, 4);

        Button.waitForAnyPress();
        // courseInfo = getBluetoothData();
        // 
        Navigation nav = new Navigation(robot, odometer, map, courseInfo);
        Localizer localizer = new Localizer(robot, odometer, map, nav, courseInfo[0]);

        // robot.leftMotor.flt();
        // robot.rightMotor.flt();

        // CLEAR DISPLAY
        LCD.clear();

        // start odometer and display
        odometer.start();
        display.start();

        // localize
        Localizer.localize();

        // search
        // nav.start();
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