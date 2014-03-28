import lejos.nxt.*;
import java.util.*;

/**
 * Localizes robot at beginning of run based on sensor readings
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 1.4
 */
public class Localizer {
    // OBJECTS
    private Robot robot;
    private Odometer odometer;
    private Navigation nav;

    // STARTING CORNER
    private int corner;

    private final int LEFT = 0, RIGHT = 1;

    // FILTER COUNTERS AND DATA
    private int[] sonicCounter = {0,0};
    private int[] lightCounter = {0,0};
    private int[][] distance = new int[2][3];
    private int[][] intensity = new int[2][10];
    private int[] sortedIntensity = new int[intensity[0].length];

    /**
     * Localizer constructor
     *
     * @param robot             robot object containing all color sensor ports
     * @param odometer          odometer object containing x, y and theta coordinates
     * @param corner            corner where robot started
     */
    public Localizer(Robot robot, Odometer odometer, Navigation nav, int corner) {
        this.robot = robot;
        this.odometer = odometer;
        this.nav = nav;
        this.corner = corner;

        // SET UP MOVING AVERAGE FILTERS
        for (int i = 0; i < distance[0].length; i++)
            distance[LEFT][i] = robot.leftSonic.getDistance();

        for (int i = 0; i < distance[0].length; i++)
            distance[RIGHT][i] = robot.rightSonic.getDistance();

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < intensity[0].length; j++)
                intensity[i][j] = 600;
    }

    /**
     * Localizes using ultrasonic sensor followed by downward facing
     * color sensors
     */
    public void localize() {
        // SELF-EXPLANATORY
        // localizeSonicly();

        // FACE 90 DEGREES
        nav.turnTo(Math.PI / 4);

        // SELF-EXPLANATORY
        localizeLightly();

        nav.travelTo(0, 0);
        nav.turnTo(Math.PI / 2);

        // CORRECT COORDINATE
        correct();
    }

    /**
     * Localizes using falling edge method with outermost ultrasonic sensor
     * and corrects odometer
     */
    public void localizeSonicly() {
        // SET LOCALIZING FLAG
        robot.localizing = true;

        double firstAngle, secondAngle, deltaTheta;

        // SET LOW SPEED TO IMPROVE ACCURACY
        nav.setMotorRotateSpeed(100);

        LCD.drawString("LOCATING...", 0, 0);

        // ROTATE THE ROBOT UNTIL IT SEES NO WALL
        robot.leftMotor.forward();
        robot.rightMotor.backward();
        while (isWall(RIGHT));

        // PLAY SOUND
        Sound.playTone(3000,100);

        // KEEP ROTATING UNTIL THE ROBOT SEES A WALL, THEN LATCH THE ANGLE
        while (!isWall(RIGHT));
        firstAngle = Math.toDegrees(odometer.getTheta());

        // PLAY LOWER FREQUENCY SOUND
        Sound.playTone(2000,100);

        // SWITCH DIRECTION AND WAIT UNTIL IT SEES NO WALL
        robot.leftMotor.backward();
        robot.rightMotor.forward();
        while (isWall(LEFT));

        // PLAY HIGHER FREQUENCY SOUND
        Sound.playTone(3000,100);

        // KEEP ROTATING UNTIL THE ROBOT SEES A WALL, THEN LATCH THE ANGLE
        while (!isWall(LEFT));
        secondAngle = Math.toDegrees(odometer.getTheta());

        // PLAY LOWER FREQUENCY SOUND
        Sound.playTone(2000,100);

        // MEASURE ORIENTATION
        deltaTheta = (secondAngle + firstAngle) / 2 - 50;

        // UPDATE THE ODOMETER POSITION
        odometer.setTheta(Math.toRadians(secondAngle - deltaTheta));

        // RESET LOCALIZING FLAG
        robot.localizing = false;
	}

    /**
     * Localizes using downward facing color sensors and corrects odometer
     */
    public void localizeLightly() {
        // SET LOCALIZING FLAG
        robot.localizing = true;
        LCD.drawString("LOCATING...", 0, 0);

        // SET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(true);
        robot.rightColor.setFloodlight(true);

        // STORE COORDINATES
        double[][] posLeft = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}};
        double[][] posRight = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}};

        // DETECT ALL FOUR LINES
        int counterLeft = 0, counterRight = 0, timerLeft = 0, timerRight = 0;
        int tachoA = 0, tachoB = 0, tachoC = 0, seenFirst = LEFT;
        boolean done = false;
        while (!done) {
            nav.setMotorSpeeds(100, 100);
            robot.leftMotor.forward();
            robot.rightMotor.forward();

            if (--timerLeft <= 0 && isLine(LEFT)){
                odometer.getPosition(posLeft[counterLeft++], new boolean[] { true, true, true });
                Sound.playTone(2000,100);
                timerLeft = 500;

                if (counterLeft == 1 && counterRight == 0) {
                    tachoA = robot.leftMotor.getTachoCount();
                    tachoC = robot.rightMotor.getTachoCount();
                    seenFirst = LEFT;
                } else if (counterLeft == 2 && counterRight == 2) {
                    tachoA = robot.leftMotor.getTachoCount() - tachoA;
                    robot.leftColor.setFloodlight(false);
                } else if (counterLeft == 1 && counterRight == 1) {
                    tachoB = robot.leftMotor.getTachoCount();
                    tachoC = robot.leftMotor.getTachoCount() - tachoC;
                } else if (counterLeft == 2 && counterRight == 1) {
                    tachoB = robot.leftMotor.getTachoCount() - tachoB;
                    robot.leftColor.setFloodlight(false);
                }
            }

            if (--timerRight <= 0 && isLine(RIGHT)) {
                odometer.getPosition(posRight[counterRight++], new boolean[] { true, true, true });
                Sound.playTone(2000,100);
                timerRight = 500;

                if (counterRight == 1 && counterLeft == 0) {
                    tachoB = robot.rightMotor.getTachoCount();
                    tachoC = robot.rightMotor.getTachoCount() - tachoC;
                    seenFirst = RIGHT;
                } else if (counterRight == 2 && counterLeft == 2) {
                    tachoA = robot.rightMotor.getTachoCount() - tachoA;
                    robot.rightColor.setFloodlight(false);
                } else if (counterRight == 1 && counterLeft == 1) {
                    tachoA = robot.rightMotor.getTachoCount();
                } else if (counterRight == 2 && counterLeft == 1) {
                    tachoB = robot.rightMotor.getTachoCount() - tachoB;
                    robot.rightColor.setFloodlight(false);
                }
            }

            if (counterLeft == 2 && counterRight == 2)
                done = true;
        }

        // STOP
        nav.stop();

        // CALCULATE
        double a, b, c, e = robot.distanceBetweenColorSensors / 2;
        double x, y, theta;

        if (seenFirst == LEFT) {
            a = tachoA * robot.leftRadius / 360;
            b = tachoB * robot.rightRadius / 360;
            c = tachoC * robot.rightRadius / 360;

            theta = Math.atan2(2 * e, b + c);
            x = e * Math.sin(theta);
            y = a * Math.sin(theta) - e * Math.cos(theta);
        } else {
            a = tachoA * robot.rightRadius / 360;
            b = tachoB * robot.leftRadius / 360;
            c = tachoC * robot.leftRadius / 360;
            theta = Math.atan2(2 * e, b + c);
            x = a * Math.cos(theta) - e * Math.sin(theta);
            y = e * Math.cos(theta);
        }
        // CORRECT ODOMETER
        odometer.setTheta(theta);
        odometer.setX(x);
        odometer.setY(y);

        // RESET LOCALIZING FLAG
        robot.localizing = false;
    }

    /**
     * Correct coordinates based on starting corner information provided
     */
    public void correct() {
        switch(corner) {
            case 1:     // (0, 0)   TILE
                break;
            case 2:     // (10, 0)  TILE
                odometer.setX(odometer.getX() + 10 * 30.48);
                odometer.setTheta(Math.PI);
                break;
            case 3:     // (10, 10) TILE
                odometer.setX(odometer.getX() + 10 * 30.48);
                odometer.setY(odometer.getY() + 10 * 30.48);
                odometer.setTheta(3 * Math.PI / 2);
                break;
            case 4:     // (0, 10) TILE
                odometer.setY(odometer.getY() + 10 * 30.48);
                odometer.setTheta(0);
                break;
        }
    }

    /**
     * Returns whether a wall is detected by the selected ultrasonic sensor
     *
     * @param side         select color sensor, 1 for RIGHT and 0 for LEFT
     *
     * @return <code>true</code> if wall detected; <code>false</code> otherwise
     */
    private boolean isWall(int side) {
        // SELECT CORRECT ULTRASONIC SENSOR
        UltrasonicSensor sonic = (side == RIGHT) ? robot.rightSonic : robot.leftSonic;

        // GET DISTANCE
        distance[side][sonicCounter[side]++] = sonic.getDistance();

        if (sonicCounter[side] == distance[0].length)
            sonicCounter[side] = 0;

        // COMPUTE AVERAGE
        int sum = 0;

        for (int i = 0; i < distance[side].length; i++)
            sum += distance[side][i];

        int avg = sum / distance[side].length;

        // ANALYZE
        if (avg < 30) {
            for (int i = 0; i < distance[side].length; i++)
                distance[side][i] = 255;
            return true;
        }
        else
            return false;
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
