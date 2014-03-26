import lejos.nxt.*;

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
    private int[][] intensity = new int[2][3];

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
                intensity[i][j] = 500;
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
        boolean done = false;
        while (!done) {
            nav.setMotorSpeeds(100, 100);
            if (isLine(LEFT)){
                odometer.getPosition(posLeft[counterLeft++], new boolean[] { true, true, true });
                Sound.playTone(2000,100);
                while(isLine(LEFT)) {
                    if (isLine(RIGHT) && counterRight < counterLeft) {
                        odometer.getPosition(posRight[counterRight++], new boolean[] {true, true, true});
                    }
                }
            }
            if (isLine(RIGHT)) {
                odometer.getPosition(posRight[counterRight++], new boolean[] { true, true, true });
                Sound.playTone(2000,100);
                while(isLine(RIGHT)) {
                    if (isLine(LEFT) && counterLeft < counterRight) {
                        odometer.getPosition(posLeft[counterLeft++], new boolean[] {true, true, true});
                        break;
                    }
                }
            }

            if (counterLeft == 2 && counterRight == 2)
                done = true;
        }

        // STOP
        nav.stop();

        // CALCULATE
        double deltaLeft = nav.distance(posLeft[0], posLeft[1]);
        double deltaRight = nav.distance(posRight[0], posRight[1]);

        double thetaLeft = (posLeft[0][2] + posLeft[1][2]) / 2;
        double thetaRight = (posRight[0][2] + posRight[1][2]) / 2;

        double deltaX = -deltaLeft * Math.cos(thetaLeft) + 5; // CONSISTENTLY OFF BY 5 CM CUZ DPM
        double deltaY = -deltaRight * Math.cos(thetaRight);

        // RESET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(false);
        robot.rightColor.setFloodlight(false);

        // CORRECT POSITION
        odometer.setX(odometer.getX() + deltaX);
        odometer.setY(odometer.getY() + deltaY);

        // RESET LOCALIZING FLAG
        robot.localizing = false;
    }

    /**
     * Correct coordinates based on starting corner information provided
     */
    public void correct() {
        switch(corner) {
            case 1: // (0, 0) TILES
                break;
            case 2: // (10, 0) TILES
                odometer.setX(odometer.getX() + 10 * 30.48);
                odometer.setTheta(Math.PI);
                break;
            case 3: // (10, 10) TILES
                odometer.setX(odometer.getX() + 10 * 30.48);
                odometer.setY(odometer.getY() + 10 * 30.48);
                odometer.setTheta(3 * Math.PI / 2);
                break;
            case 4: // (0, 10) TILES
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
        if (avg < 30)
            return true;
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
