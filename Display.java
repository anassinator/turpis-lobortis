import lejos.nxt.*;

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

	/**
	 * Display constructor
   *
	 * @param odometer         odometer object containing robot's position and orientation
	 */
	public Display(Odometer odometer, Robot robot) {
		this.odometer = odometer;
        this.robot = robot;

        robot.leftSonic.continuous();
        robot.centerSonic.continuous();
        robot.rightSonic.continuous();
	}

	/**
	 * Peridically updates odometer data on the screen
	 */
	public void run() {
        long displayStart, displayEnd;
        double[] position = new double[3];

        robot.leftColor.setFloodlight(true);
        robot.rightColor.setFloodlight(true);

        while (true) {
            displayStart = System.currentTimeMillis();

            odometer.getPosition(position, new boolean[] { true, true, true });

            print(position);

            // throttle the OdometryDisplay
            displayEnd = System.currentTimeMillis();
            if (displayEnd - displayStart < DISPLAY_PERIOD) {
                try {
                    Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
                } catch (InterruptedException e) {
                    // there is nothing to be done here because it is not
                    // expected that OdometryDisplay will be interrupted
                    // by another thread
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

        LCD.drawString("LS: ", 0, 3);
        LCD.drawString("CS: ", 0, 4);
        LCD.drawString("RS: ", 0, 5);
        LCD.drawString(format(robot.leftSonic.getDistance()), 4, 3);
        LCD.drawString(format(robot.centerSonic.getDistance()), 4, 4);
        LCD.drawString(format(robot.rightSonic.getDistance()), 4, 5);

        LCD.drawString("LC: ", 0, 6);
        LCD.drawString("RC: ", 0, 7);
        LCD.drawString(String.valueOf(robot.leftColor.getNormalizedLightValue()), 4, 6);
        LCD.drawString(String.valueOf(robot.rightColor.getNormalizedLightValue()), 4, 7);
	}

    private String format(int x) {
        if (x / 100 > 0)
            return String.valueOf(x);
        else if (x / 10 > 0)
            return "0" + String.valueOf(x);
        else
            return "00" + String.valueOf(x);
    }
}
