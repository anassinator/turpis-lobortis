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

	/**
	 * Display constructor
   *
	 * @param odometer         odometer object containing robot's position and orientation
	 */
	public Display(Odometer odometer) {
		this.odometer = odometer;
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
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawString(String.valueOf(Math.round(position[0] * 100.0) / 100.0), 3, 0);
		LCD.drawString(String.valueOf(Math.round(position[1] * 100.0) / 100.0), 3, 1);
		LCD.drawString(String.valueOf(Math.round(position[2] * 100.0) / 100.0), 3, 2);
	}
}
