import lejos.nxt.*;
import lejos.robotics.*;

/**
 * Displays robot's position and orientation
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.1
 */
public class Display extends Thread {
	private Odometer odometer;
	private double [] pos;
    private static final long DISPLAY_PERIOD = 20;
    private Robot robot;
    private boolean debugging;

	/**
	 * Display constructor
     *
	 * @param odometer         odometer object containing robot's position and orientation
     * @param robot            robot object containing all sensors
     * @param debugging        whether to display debugging information
	 */
	public Display(Odometer odometer, Robot robot, boolean debugging) {
		this.odometer = odometer;
        this.robot = robot;
        this.debugging = debugging;

        // SET UP DEBUGGING INTERFACE
        if (debugging) {
            robot.leftSonic.continuous();
            robot.centerSonic.continuous();
            robot.rightSonic.continuous();
            robot.leftColor.setFloodlight(true);
            robot.rightColor.setFloodlight(true);
        }
	}

	/**
	 * Peridically updates odometer data on the screen
	 */
	public void run() {
        long displayStart, displayEnd;
        double[] position = new double[3];

        while (true) {
            displayStart = System.currentTimeMillis();

            odometer.getPosition(position, new boolean[] { true, true, true });

            print(position);

            // THROTTLE THE ODOMETRYDISPLAY
            displayEnd = System.currentTimeMillis();
            if (displayEnd - displayStart < DISPLAY_PERIOD) {
                try {
                    Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
                } catch (InterruptedException e) {
                    // THERE IS NOTHING TO BE DONE HERE BECAUSE IT IS NOT
                    // EXPECTED THAT ODOMETRYDISPLAY WILL BE INTERRUPTED
                    // BY ANOTHER THREAD
                }
            }
        }
	}

	/**
	 * Prints the robot's x and y coordinates in centimeters
	 * and orientation in degrees
	 */
	public void print(double[] position) {
		// LCD.clear();
		LCD.drawString("X:  ", 0, 0);
		LCD.drawString("Y:  ", 0, 1);
		LCD.drawString("H:  ", 0, 2);
		LCD.drawString(String.valueOf(Math.round(position[0] * 100.0) / 100.0), 4, 0);
		LCD.drawString(String.valueOf(Math.round(position[1] * 100.0) / 100.0), 4, 1);
		LCD.drawString(String.valueOf(Math.round(position[2] * 100.0) / 100.0), 4, 2);

        LCD.drawString(format((int)(position[0] / 30.48)), 13, 0);
        LCD.drawString(format((int)(position[1] / 30.48)), 13, 1);

        LCD.drawString(" ", 13, 0);
        LCD.drawString(" ", 13, 1);

        if (debugging) {
            LCD.drawString("LS: ", 0, 3);
            LCD.drawString("CS: ", 0, 4);
            LCD.drawString("RS: ", 0, 5);
            LCD.drawString(format(robot.leftSonic.getDistance()), 4, 3);
            LCD.drawString(format(robot.centerSonic.getDistance()), 4, 4);
            LCD.drawString(format(robot.rightSonic.getDistance()), 4, 5);

            LCD.drawString("LC: ", 0, 6);
            LCD.drawString("RC: ", 0, 7);
            LCD.drawString(format(robot.leftColor.getRawLightValue()), 4, 6);
            LCD.drawString(format(robot.rightColor.getRawLightValue()), 4, 7);
        }
	}

    /**
     * Returns a fixed width formatted string to fit the display nicely
     *
     * @param x         integer to format
     *
     * @return the formatted number
     */
    private String format(int x) {
        if (x / 100 > 0)
            return String.valueOf(x);
        else if (x / 10 > 0)
            return "0" + String.valueOf(x);
        else
            return "00" + String.valueOf(x);
    }
}
