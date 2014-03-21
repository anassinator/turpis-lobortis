import lejos.nxt.*;
import lejos.util.*;

/**
 * Controller for master NXT
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 1.0
 */
public class Turpis {
    public static int[] courseInfo = { 1 };

    public static Robot robot;
    public static Odometer odometer;
    public static Map map;
    public static Display display;
    public static boolean truthful = true;

    public static void main(String[] args) {
        // SET VOLUME TO MAX
        Sound.setVolume(Sound.VOL_MAX);

        LCD.drawString("WELCOME", 5, 4);

        Button.waitForAnyPress();

        robot = new Robot();

        // Recognition recognizer = new Recognition(robot);
        // recognizer.recognize();

        Claw claw = new Claw(robot);
        claw.grab();

        while (truthful) {
            claw.drop();
            Delay.msDelay(500);
            claw.grab();
            Delay.msDelay(500);
        }

        odometer = new Odometer(robot);
        // map = new Map(robot, odometer);
        display = new Display(odometer);

        // courseInfo = getBluetoothData();

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
        localizer.localize();

        // Correction corrector = new Correction(robot, odometer);
        // corrector.start();

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