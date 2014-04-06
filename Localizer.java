import lejos.nxt.*;
import java.util.*;
import lejos.robotics.*;

/**
 * Localizes robot at beginning of run based on sensor readings
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 1.5
 */
public class Localizer {
    // OBJECTS
    private Robot robot;
    private Odometer odometer;
    private Navigation nav;

    // STARTING CORNER
    private int corner;

    private final int LEFT = 0, RIGHT = 1;
    private final double SIZE_OF_TILE = 30.48;

    // FILTER COUNTERS AND DATA
    private int[] sonicCounter = {0,0};
    private int[] lightCounter = {0,0};
    private int[][] distance = new int[2][3];
    private int[][] intensity = new int[2][5];
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
        localizeSonicly();

        // FACE 90 DEGREES
        nav.turnTo(Math.PI / 2);

        // SELF-EXPLANATORY
        localizeLightly();

        nav.travelTo(0, 0);
        nav.turnTo(Math.PI / 2);

        // CORRECT COORDINATE
        correct();
    }

    /**
     * Relocalizes using downward facing color sensors
     */
    public void relocalize() {
        double[] desiredPos = {0.0, 0.0, 0.0};

        // SET LOCALIZING FLAG
        robot.localizing = true;
        LCD.drawString("LOCATING...", 0, 0);

        // START
        nav.turnTo(Math.PI / 2);

        // MAKE SURE NOT TOO CLOSE TO LINE
        nav.goBackward(10);

        // SET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(true);
        robot.rightColor.setFloodlight(true);

        if (odometer.getY() < 0)
            desiredPos[1] = (int)(odometer.getY() / SIZE_OF_TILE) * SIZE_OF_TILE;
        else
            desiredPos[1] = (int)(odometer.getY() / SIZE_OF_TILE) * SIZE_OF_TILE + SIZE_OF_TILE;

        // GO FORWARD
        robot.leftMotor.forward();
        robot.rightMotor.forward();
        robot.leftMotor.setSpeed(250);
        robot.rightMotor.setSpeed(250);

        // THIS WILL MAKE THE ROBOT STOP ON THE X AXIS CORRECTING THE Y COORDINATES
        // WILL MOVE EACH TIRE UNTIL IT REACHES A GRIDLINE, STOPPING DIRECTLY ON THE LINE
        boolean leftDone = false, rightDone = false;
        while (!leftDone || !rightDone) {
            if (!leftDone && isLine(LEFT)) {
                Sound.beep();
                robot.leftMotor.stop(true);
                leftDone = true;
            }
            if (!rightDone && isLine(RIGHT)) {
                Sound.beep();
                robot.rightMotor.stop(true);
                rightDone = true;
            }
        }

        // SETS THE ODOMETER AT 90 DEGREES, THE ACTUAL ORIENTATION OF THE ROBOT
        odometer.setPosition(new double[] {odometer.getX(), desiredPos[1], Math.PI / 2 - Math.toRadians(5) },
                             new boolean[] {true, true, true});

        // TURN TO 0 DEGREES TO FIX THE X COORDINATES
        nav.turnTo(0);

        // MAKE SURE NOT TOO CLOSE TO LINE
        nav.goBackward(10);

        if (odometer.getX() < 0)
            desiredPos[0] = (int)(odometer.getX() / SIZE_OF_TILE) * SIZE_OF_TILE;
        else
            desiredPos[0] = (int)(odometer.getX() / SIZE_OF_TILE) * SIZE_OF_TILE + SIZE_OF_TILE;

        // GO FORWARD
        robot.leftMotor.forward();
        robot.rightMotor.forward();
        robot.leftMotor.setSpeed(250);
        robot.rightMotor.setSpeed(250);

        // THIS WILL MAKE THE ROBOT STOP ON THE Y AXIS CORRECTING THE X COORDINATES
        // WILL MOVE EACH TIRE UNTIL IT REACHES A GRIDLINE, STOPPING DIRECTLY ON THE LINE
        leftDone = false;
        rightDone = false;
        while (!leftDone || !rightDone) {
            if (!leftDone && isLine(LEFT)) {
                Sound.beep();
                robot.leftMotor.stop(true);
                leftDone = true;
            }
            if (!rightDone && isLine(RIGHT)) {
                Sound.beep();
                robot.rightMotor.stop(true);
                rightDone = true;
            }
        }

        // RESET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(false);
        robot.rightColor.setFloodlight(false);

        // SETS THE ODOMETER AT 0 DEGREES, THE ACTUAL ORIENTATION OF THE ROBOT
        odometer.setPosition(new double[] {desiredPos[0] - 4.2, desiredPos[1] - 4.2, -Math.toRadians(5)},
                             new boolean[] {true, true, true});

        // RESET LOCALIZING FLAG
        robot.localizing = false;
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
        nav.setMotorRotateSpeed(480);

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

        // GO FORWARD
        robot.leftMotor.forward();
        robot.rightMotor.forward();
        robot.leftMotor.setSpeed(250);
        robot.rightMotor.setSpeed(250);

        // THIS WILL MAKE THE ROBOT STOP ON THE X AXIS CORRECTING THE Y COORDINATES
        // WILL MOVE EACH TIRE UNTIL IT REACHES A GRIDLINE, STOPPING DIRECTLY ON THE LINE
        boolean leftDone = false, rightDone = false;
        while (!leftDone || !rightDone) {
            if (!leftDone && isLine(LEFT)) {
                Sound.beep();
                robot.leftMotor.stop(true);
                leftDone = true;
            }
            if (!rightDone && isLine(RIGHT)) {
                Sound.beep();
                robot.rightMotor.stop(true);
                rightDone = true;
            }
        }

        // SETS THE ODOMETER AT 90 DEGREES, THE ACTUAL ORIENTATION OF THE ROBOT
        odometer.setPosition(new double[] {0, 0, Math.PI / 2 - Math.toRadians(5)},
                             new boolean[] {true, true, true});

        // TURN TO 0 DEGREES TO FIX THE X COORDINATES
        nav.turnTo(0);

        // GO FORWARD
        robot.leftMotor.forward();
        robot.rightMotor.forward();
        robot.leftMotor.setSpeed(250);
        robot.rightMotor.setSpeed(250);

        // THIS WILL MAKE THE ROBOT STOP ON THE Y AXIS CORRECTING THE X COORDINATES
        // WILL MOVE EACH TIRE UNTIL IT REACHES A GRIDLINE, STOPPING DIRECTLY ON THE LINE
        leftDone = false;
        rightDone = false;
        while (!leftDone || !rightDone) {
            if (!leftDone && isLine(LEFT)) {
                Sound.beep();
                robot.leftMotor.stop(true);
                leftDone = true;
            }
            if (!rightDone && isLine(RIGHT)) {
                Sound.beep();
                robot.rightMotor.stop(true);
                rightDone = true;
            }
        }

        // RESET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(false);
        robot.rightColor.setFloodlight(false);

        // SETS THE ODOMETER AT 0 DEGREES, THE ACTUAL ORIENTATION OF THE ROBOT
        odometer.setPosition(new double[] {-4.2, -4.2, -Math.toRadians(5)},
                             new boolean[] {true, true, true});

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
    public boolean isLine(int side) {
        // SELECT CORRECT COLOR SENSOR
        ColorSensor color = (side == RIGHT) ? robot.rightColor : robot.leftColor;

        // GET LIGHT VALUE
        intensity[side][lightCounter[side]++] = color.getRawLightValue();

        if (lightCounter[side] == intensity[side].length)
            lightCounter[side] = 0;

        for (int i = 0; i < intensity[side].length; i++)
            sortedIntensity[i] = intensity[side][i];

        Arrays.sort(sortedIntensity);

        // COMPUTE AVERAGE MEDIAN
        int sum = 0;

        for (int i = 1; i < sortedIntensity.length - 1; i++)
            sum += sortedIntensity[i];

        int median = sum / (sortedIntensity.length - 2);

        // ANALYZE
        if (median <= 400) {
            for (int i = 0; i < intensity[side].length; i++)
                intensity[side][i] = 600;

            return true;
        }
        // if (color.getRawLightValue() < 360)
        //     return true;

        return false;
    }

}
