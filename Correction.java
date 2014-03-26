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
    }

    /**
     * Corrects odometer using downward facing color sensors based on angle of attack
     */
    public void run() {
        while (true) {
            // STORE COORDINATES
            double firstAngle, secondAngle, deltaTheta;
            double[] posLeft = {0.0, 0.0, 0.0};
            double[] posRight = {0.0, 0.0, 0.0};

            // DETECT BOTH LINES
            int timerLeft = 0, timerRight = 0, counter = 0, lastDone = 0;
            boolean done = false;
            while (!done) {
                if (--timerLeft <= 0 && isLine(LEFT)){
                    odometer.getPosition(posLeft, new boolean[] { true, true, true });
                    Sound.playTone(2000,100);
                    timerLeft = 500;
                    counter++;
                    lastDone = LEFT;
                }

                if (--timerRight <= 0 && isLine(RIGHT)) {
                    odometer.getPosition(posRight, new boolean[] { true, true, true });
                    Sound.playTone(2000,100);
                    timerRight = 500;
                    counter++;
                    lastDone = RIGHT;
                }

                if (counter == 2)
                    done = true;
            }

            // CALCULATE
            double distance = distance(posLeft, posRight);
            double theta = Math.asin(robot.distanceBetweenColorSensors / distance);

            // CORRECT FOR DIRECTION
            if (lastDone == RIGHT)
                theta = Math.PI - theta;
            switch (direction(posLeft)) {
                case NORTH:
                    break;
                case WEST:
                    theta += Math.PI / 2;
                    break;
                case SOUTH:
                    theta += Math.PI;
                    break;
                case EAST:
                    theta -= Math.PI / 2;
                    break;
            }

            // SET ODOMETER
            odometer.setTheta(theta);
        }
    }

    /**
     * Returns direction robot faces
     *
     * <TABLE BORDER=1>
     * <TR><TH>Direction Code</TH><TH>Direction</TH></TR>
     * <TR><TD>0</TD><TD>North</TD></TR>
     * <TR><TD>1</TD><TD>West</TD></TR>
     * <TR><TD>2</TD><TD>South</TD></TR>
     * <TR><TD>3</TD><TD>East</TD></TR>\
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

        if (lightCounter[side] == intensity[0].length)
            lightCounter[side] = 0;

        // COMPUTE AVERAGE
        int sum = 0;

        for (int i = 0; i < intensity[side].length; i++)
            sum += intensity[side][i];

        int avg = sum / intensity[side].length;

        // ANALYZE
        if (avg <= 350)
            return true;
        else
            return false;
    }

}
