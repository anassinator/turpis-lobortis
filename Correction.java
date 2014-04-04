import lejos.nxt.*;

// TODO: FIX AND TEST

/**
 * Corrects odometry based on line readings from downward facing
 * color sensors
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.5
 */
public class Correction extends Thread {
    // OBJECTS
    private Robot robot;
    private Odometer odometer;

    // DEFINES
    private static final int NORTH = 0, WEST = 1, SOUTH = 2, EAST = 3;
    private static final int LEFT = 0, RIGHT = 1;

    // FILTER COUNTERS AND DATA
    private int[] lightCounter = {0,0};
    private int[][] intensity = new int[2][10];
    private int[] sortedIntensity = new int[intensity[0].length];

    /**
     * Correction constructor
     *
     * @param robot             robot object containing all color sensor ports
     * @param odometer          odometer object containing x, y and theta coordinates
     */
    public Correction(Robot robot, Odometer odometer) {
        this.robot = robot;
        this.odometer = odometer;

        // SET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(true);
        robot.rightColor.setFloodlight(true);

        // SET UP MOVING AVERAGE FILTER
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < intensity[0].length; j++)
                intensity[i][j] = 600;
    }

    /**
     * Corrects odometer using downward facing color sensors based on angle of attack
     */
    public void run() {
        while (true) {
            // COUNTERS
            int tachoLeft = 0, tachoRight = 0;
            int timerLeft = 0, timerRight = 0;
            int counterLeft = 0, counterRight = 0;
            int lastDone = 0;

            // DETECT BOTH LINES
            boolean done = false;
            while (!done) {
                if (--timerLeft < 0 && counterLeft < 2 && isLine(LEFT)) {
                    tachoLeft = robot.leftMotor.getTachoCount() - tachoLeft;
                    counterLeft++;
                    timerLeft = 500;
                    lastDone = LEFT;
                }

                if (--timerRight < 0 && counterRight < 2 && isLine(RIGHT)) {
                    tachoRight = robot.rightMotor.getTachoCount() - tachoRight;
                    counterRight++;
                    timerRight = 500;
                    lastDone = RIGHT;
                }

                if (counterLeft == 2 && counterRight == 2)
                    done = true;
            }

            // COMPUTE
            double distanceLeft = tachoLeft * robot.leftRadius / 360;
            double distanceRight = tachoRight * robot.rightRadius / 360;
            double distance = 0;

            switch (crossover()) {
                case CROSS:
                    distance = distanceLeft + distanceRight;
                    break;
                case LEFT:
                    distance = distanceLeft - distanceRight;
                    break;
                case RIGHT:
                    distance = distanceRight - distanceLeft;
                    break;
            }

            double theta = Math.asin(2 * robot.distanceBetweenColorSensors / distance) / 2;

            // SET ODOMETER
            odometer.setTheta(theta);
        }
    }

    /**
     * Returns direction robot faces
     *
     * <TABLE BORDER=1>
     * <TR><TH>Direction Code</TH><TH>Direction</TH></TR>
     * <TR><TD>0</TD><TD>LEFT</TD></TR>
     * <TR><TD>1</TD><TD>RIGHT</TD></TR>
     * <TR><TD>2</TD><TD>CROSS</TD></TR>
     * </TABLE>
     *
     * @return the direction code
     */
    public int crossover(double[] pos) {
        double orientation = 4 * pos[2];
        if (orientation >= Math.PI && orientation < 3 * Math.PI)
            return NORTH;
        else if (orientation >= 3 * Math.PI && orientation < 5 * Math.PI)
            return WEST;
        else if (orientation >= 5 * Math.PI && orientation < 7 * Math.PI)
            return SOUTH;
        else
            return EAST;

    }

    /**
     * Returns direction robot faces
     *
     * <TABLE BORDER=1>
     * <TR><TH>Direction Code</TH><TH>Direction</TH></TR>
     * <TR><TD>0</TD><TD>North</TD></TR>
     * <TR><TD>1</TD><TD>West</TD></TR>
     * <TR><TD>2</TD><TD>South</TD></TR>
     * <TR><TD>3</TD><TD>East</TD></TR>
     * </TABLE>
     *
     * @return the direction code
     */
    public int direction(double[] pos) {
        double orientation = 4 * pos[2];
        if (orientation >= Math.PI && orientation < 3 * Math.PI)
            return NORTH;
        else if (orientation >= 3 * Math.PI && orientation < 5 * Math.PI)
            return WEST;
        else if (orientation >= 5 * Math.PI && orientation < 7 * Math.PI)
            return SOUTH;
        else
            return EAST;

    }

    /**
     * Returns distance between two sets of coordinates passed in an array
     *
     * @param firstPosition         first array of X and Y coordinates
     * @param secondPosition        second array of X and Y coordinates
     *
     * @return the distance in centimeters
     */
    public double distance(double[] firstPosition, double[] secondPosition) {
        return Math.sqrt(Math.pow(secondPosition[0] - firstPosition[0], 2) + Math.pow(secondPosition[1] - firstPosition[1], 2));
    }

    /**
     * Returns whether a line is under the selected color sensor is above a
     * a black line
     *
     * @param side         select color sensor, 1 for RIGHT and 0 for LEFT
     *
     * @return <code>true</code> if line detected; <code>false</code> otherwise
     */
    private boolean isLine(int side) {
        // SELECT CORRECT COLOR SENSOR
        ColorSensor color = (side == RIGHT) ? robot.rightColor : robot.leftColor;

        // GET LIGHT VALUE
        intensity[side][lightCounter[side]++] = color.getNormalizedLightValue();

        if (lightCounter[side] == intensity[side].length)
            lightCounter[side] = 0;

        for (int i = 0; i < intensity[side].length; i++)
            sortedIntensity[i] = intensity[side][i];

        Arrays.sort(sortedIntensity);

        // COMPUTE AVERAGE MEDIAN
        int sum = 0;

        for (int i = 3; i < sortedIntensity.length - 3; i++)
            sum += sortedIntensity[i];

        int median = sum / (sortedIntensity.length - 6);

        // ANALYZE
        if (median <= 500) {
            for (int i = 0; i < intensity[side].length; i++)
                intensity[side][i] = 600;

            return true;
        }

        return false;
    }

}
