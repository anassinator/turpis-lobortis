import lejos.nxt.*;
import java.util.*;
import lejos.robotics.*;

/**
 * Localizes robot at beginning of run based on sensor readings
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 2.0
 */
public class Localizer {
    // OBJECTS
    private Robot robot;
    private Odometer odometer;
    private Navigation nav;

    // STARTING CORNER
    private int corner;

    private final int LEFT = 0, RIGHT = 1, FRONT = 2;
    private final double SIZE_OF_TILE = 30.48;

    // FILTER COUNTERS AND DATA
    private int[] sonicCounter = {0,0,0};
    private int[] lightCounter = {0,0};
    private int[] lineCounter = {0,0};
    private int[][] distance = new int[3][3];
    private int[][] intensity = new int[2][10];
    private int[] threshold = {450, 420};
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

        for (int i = 0; i < distance[0].length; i++)
            distance[FRONT][i] = robot.centerSonic.getDistance();

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

        nav.turnTo(Math.PI / 2);

        // SELF-EXPLANATORY
        localizeLightly();

        // CORRECT COORDINATE
        correct();
    }

    /**
     * Returns whether a color sensor is straight on a line or not
     *
     * <TABLE BORDER=1>
     * <TR><TH>Return Code</TH><TH>Description</TH></TR>
     * <TR><TD>-1</TD><TD>Not on a line</TD></TR>
     * <TR><TD>0</TD><TD>Left color sensor on a line</TD></TR>
     * <TR><TD>1</TD><TD>Right color sensor on a line/TD></TR>
     * </TABLE>
     *
     * @return the return code
     */
    public int isStraightOnLine() {
        double distanceToCheck = 5.00;

        robot.leftMotor.rotate(-convertDistance(robot.leftRadius, distanceToCheck), true);
        robot.rightMotor.rotate(-convertDistance(robot.rightRadius, distanceToCheck), true);

        while (robot.leftMotor.isMoving() || robot.rightMotor.isMoving()) {
            for (int side = 0; side < 2; side++) {
                if (isLine(side)) {
                    lineCounter[side]++;
                }
            }
        }

        robot.leftMotor.rotate(convertDistance(robot.leftRadius, distanceToCheck), true);
        robot.rightMotor.rotate(convertDistance(robot.rightRadius, distanceToCheck), true);

        while (robot.leftMotor.isMoving() || robot.rightMotor.isMoving()) {
            for (int side = 0; side < 2; side++) {
                if (isLine(side)) {
                    lineCounter[side]++;
                }
            }
        }

        if (lineCounter[LEFT] > 150) {
            lineCounter[LEFT] = 0;
            lineCounter[RIGHT] = 0;
            return LEFT;
        }

        if (lineCounter[RIGHT] > 150) {
            lineCounter[LEFT] = 0;
            lineCounter[RIGHT] = 0;
            return RIGHT;
        }

        return -1;
    }

    /**
     * Relocalizes using downward facing color sensors
     */
    public void relocalize() {
        // INITIALIZE
        double[] desiredPos = {0.0, 0.0, 0.0};
        double[] prevPos = {0.0, 0.0, 0.0};
        double[] curPos = {0.0, 0.0, 0.0};
        double delta = 0.00;

        // SET LOCALIZING FLAG
        robot.localizing = true;
        LCD.drawString("LOCATING...", 0, 7);

        // SET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(true);
        robot.rightColor.setFloodlight(true);

        // START
        nav.turnTo(Math.PI / 2);

        // MAKE SURE NOT STRAIGHT ON LINE
        int problematic = isStraightOnLine();
        if (problematic == LEFT) {
            nav.turnTo(Math.PI);
            nav.goForward(3);
            nav.turnTo(Math.PI / 2);
        } else if (problematic == RIGHT) {
            nav.turnTo(0);
            nav.goForward(3);
            nav.turnTo(Math.PI / 2);
        }

        boolean leftDone = false, rightDone = false;

        if (prevPos[1] < (nav.SIZE_OF_FIELD - 2.5) * SIZE_OF_TILE) {
            // MAKE SURE NOT ON LINE
            nav.goBackward(10);

            // GET CURRENT POSITION
            odometer.getPosition(prevPos, new boolean[] {true, true, true});

            // CORRECT FOR NEGATIVE Y
            if (prevPos[1] < 0)
                desiredPos[1] = (int)(prevPos[1] / SIZE_OF_TILE) * SIZE_OF_TILE;
            else
                desiredPos[1] = (int)(prevPos[1] / SIZE_OF_TILE) * SIZE_OF_TILE + SIZE_OF_TILE;

            // GO FORWARD
            robot.leftMotor.forward();
            robot.rightMotor.forward();
            robot.leftMotor.setSpeed(250);
            robot.rightMotor.setSpeed(250);

            // THIS WILL MAKE THE ROBOT STOP ON THE X AXIS CORRECTING THE Y COORDINATES
            // WILL MOVE EACH TIRE UNTIL IT REACHES A GRIDLINE, STOPPING DIRECTLY ON THE LINE
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

            // GET CURRENT POSITION
            odometer.getPosition(curPos, new boolean[] {true, true, true});

            // GET DISTANCE
            double distanceTraveled = nav.distance(curPos, prevPos);

            // FIX DESIRED POSITION
            if (distanceTraveled > SIZE_OF_TILE / 2) {
                desiredPos[1] += SIZE_OF_TILE;
            }

        } else {
            // GET POSITION
            odometer.getPosition(prevPos, new boolean[] {true, true, true});

            // GO FORWARD
            robot.leftMotor.backward();
            robot.rightMotor.backward();
            robot.leftMotor.setSpeed(250);
            robot.rightMotor.setSpeed(250);

            // THIS WILL MAKE THE ROBOT STOP ON THE X AXIS CORRECTING THE Y COORDINATES
            // WILL MOVE EACH TIRE UNTIL IT REACHES A GRIDLINE, STOPPING DIRECTLY ON THE LINE
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

            // GET CURRENT POSITION
            odometer.getPosition(curPos, new boolean[] {true, true, true});

            // GET DISTANCE
            double distanceTraveled = nav.distance(curPos, prevPos);

            // FIX DESIRED POSITION
            if (distanceTraveled > SIZE_OF_TILE / 2) {
                desiredPos[1] -= SIZE_OF_TILE;
            }
        }

        if (Math.abs(odometer.getTheta() - Math.PI / 2) <= Math.toRadians(15)) {
            // SETS THE ODOMETER AT 90 DEGREES IF ACCEPTABLE
            odometer.setPosition(new double[] {curPos[0], desiredPos[1], Math.PI / 2 },
                                 new boolean[] {true, true, true});
        } else {
            // RECORRECT OTHERWISE
            // relocalize();
            // return;
        }

        // GET NEW CURRENT POSITION
        odometer.getPosition(prevPos, new boolean[] {true, true, true});

        // TURN TO 0 DEGREES TO FIX THE X COORDINATES
        robot.leftMotor.rotate(-convertAngle(robot.leftTurningRadius, -Math.PI / 2), true);
        robot.rightMotor.rotate(convertAngle(robot.rightTurningRadius, -Math.PI / 2), false);

        // MAKE SURE NOT STRAIGHT ON LINE
        problematic = isStraightOnLine();
        if (problematic == LEFT) {
            nav.turnTo(Math.PI / 2);
            nav.goForward(3);
            delta += 3.00;
            nav.turnTo(0);
        } else if (problematic == RIGHT) {
            nav.turnTo(3 * Math.PI / 2);
            nav.goForward(3);
            delta -= 3.00;
            nav.turnTo(0);
        }

        // RESET FLAGS
        leftDone =  false; rightDone = false;

        if (prevPos[0] < (nav.SIZE_OF_FIELD - 2.5) * SIZE_OF_TILE) {
            // MAKE SURE NOT ON LINE
            nav.goBackward(10);

            // GET CURRENT POSITION
            odometer.getPosition(prevPos, new boolean[] {true, true, true});

            // CORRECT FOR NEGATIVE Y
            if (prevPos[0] < 0)
                desiredPos[0] = (int)(prevPos[0] / SIZE_OF_TILE) * SIZE_OF_TILE;
            else
                desiredPos[0] = (int)(prevPos[0] / SIZE_OF_TILE) * SIZE_OF_TILE + SIZE_OF_TILE;

            // GO FORWARD
            robot.leftMotor.forward();
            robot.rightMotor.forward();
            robot.leftMotor.setSpeed(250);
            robot.rightMotor.setSpeed(250);

            // THIS WILL MAKE THE ROBOT STOP ON THE X AXIS CORRECTING THE Y COORDINATES
            // WILL MOVE EACH TIRE UNTIL IT REACHES A GRIDLINE, STOPPING DIRECTLY ON THE LINE
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

            // GET CURRENT POSITION
            odometer.getPosition(curPos, new boolean[] {true, true, true});

            // GET DISTANCE
            double distanceTraveled = nav.distance(curPos, prevPos);

            // FIX DESIRED POSITION
            if (distanceTraveled > SIZE_OF_TILE / 2) {
                desiredPos[0] += SIZE_OF_TILE;
            }

        } else {
            // GET POSITION
            odometer.getPosition(prevPos, new boolean[] {true, true, true});

            // GO FORWARD
            robot.leftMotor.backward();
            robot.rightMotor.backward();
            robot.leftMotor.setSpeed(250);
            robot.rightMotor.setSpeed(250);

            // THIS WILL MAKE THE ROBOT STOP ON THE X AXIS CORRECTING THE Y COORDINATES
            // WILL MOVE EACH TIRE UNTIL IT REACHES A GRIDLINE, STOPPING DIRECTLY ON THE LINE
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

            // GET CURRENT POSITION
            odometer.getPosition(curPos, new boolean[] {true, true, true});

            // GET DISTANCE
            double distanceTraveled = nav.distance(curPos, prevPos);

            // FIX DESIRED POSITION
            if (distanceTraveled > SIZE_OF_TILE / 2) {
                desiredPos[0] -= SIZE_OF_TILE;
            }
        }

        // RESET COLOR SENSOR FLOODLIGHT
        robot.leftColor.setFloodlight(false);
        robot.rightColor.setFloodlight(false);

        if (Math.abs(odometer.getTheta()) <= Math.toRadians(35)) {
            // SETS THE ODOMETER AT 0 DEGREES IF CORRECT
            odometer.setPosition(new double[] {desiredPos[0] - 4.2, desiredPos[1] - 4.2 + delta, 0},
                                 new boolean[] {true, true, true});
        } else {
            // RECORRECT OTHERWISE
            relocalize();
            return;
        }

        // RESET LOCALIZING FLAG
        robot.localizing = false;

        LCD.drawString("           ", 0, 7);

    }

    /**
     * Localizes using falling edge method with outermost ultrasonic sensor
     * and corrects odometer
     */
    public void localizeSonicly() {
        // SET LOCALIZING FLAG
        robot.localizing = true;
        LCD.drawString("LOCATING...", 0, 7);

        // nav.setMotorRotateSpeed(480);

        // if (isWall(FRONT)) {
        //     if (isWall(LEFT)) {
        //         LCD.drawInt(1, 0, 6);
        //         robot.leftMotor.forward();
        //         robot.rightMotor.backward();
        //         while (isWall(RIGHT));
        //         Sound.beep();
        //         nav.turn(-Math.PI / 3);
        //     } else {
        //         LCD.drawInt(2, 0, 6);
        //         robot.leftMotor.backward();
        //         robot.rightMotor.forward();
        //         while (isWall(LEFT));
        //         Sound.beep();
        //         nav.turn(-Math.PI / 6);
        //     }
        // } else {
        //     if (isWall(LEFT)) {
        //         LCD.drawInt(3, 0, 6);
        //         robot.leftMotor.forward();
        //         robot.rightMotor.backward();
        //         while (isWall(LEFT));
        //         Sound.beep();
        //         nav.turn(Math.PI / 6);
        //     } else {
        //         LCD.drawInt(4, 0, 6);
        //         robot.leftMotor.backward();
        //         robot.rightMotor.forward();
        //         while (!isWall(LEFT));
        //         Sound.beep();
        //         nav.turn(-Math.PI / 6);
        //     }
        // }

        double firstAngle, secondAngle, deltaTheta;

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

        LCD.drawString("           ", 0, 7);

        // STOP
        nav.stop();
	}

    /**
     * Localizes using downward facing color sensors and corrects odometer
     */
    public void localizeLightly() {
        // SET LOCALIZING FLAG
        robot.localizing = true;
        LCD.drawString("LOCATING...", 0, 7);

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

        // SETS THE ODOMETER AT 90 DEGREES
        odometer.setPosition(new double[] {0, 0, Math.PI / 2},
                             new boolean[] {true, true, true});

        // TURN TO 0 DEGREES TO FIX THE X COORDINATES
        robot.leftMotor.rotate(-convertAngle(robot.leftTurningRadius, -Math.PI / 2), true);
        robot.rightMotor.rotate(convertAngle(robot.rightTurningRadius, -Math.PI / 2), false);

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

        // SETS THE ODOMETER AT 0 DEGREES
        odometer.setPosition(new double[] {-4.2, -4.2, 0},
                             new boolean[] {true, true, true});

        // RESET LOCALIZING FLAG
        robot.localizing = false;

        LCD.drawString("           ", 0, 7);
    }

    /**
     * Correct coordinates based on starting corner information provided
     */
    public void correct() {
        switch(corner) {
            case 1:     // (0, 0)   TILE
                break;
            case 2:     // (10, 0)  TILE
                odometer.setX(odometer.getX() + (nav.SIZE_OF_FIELD - 2) * SIZE_OF_TILE);
                odometer.setTheta(Math.PI / 2);
                break;
            case 3:     // (10, 10) TILE
                odometer.setX(odometer.getX() + (nav.SIZE_OF_FIELD - 2) * SIZE_OF_TILE);
                odometer.setY(odometer.getY() + (nav.SIZE_OF_FIELD - 2) * SIZE_OF_TILE);
                odometer.setTheta(Math.PI);
                break;
            case 4:     // (0, 10) TILE
                odometer.setY(odometer.getY() + (nav.SIZE_OF_FIELD - 2) * SIZE_OF_TILE);
                odometer.setTheta(3 * Math.PI / 2);
                break;
        }
    }

    /**
     * Computes the angle each wheel should rotate
     * in order to travel a certain distance
     *
     * @param radius        radius in centimeters
     * @param distance      distance in centimeters
     *
     * @return angle (in degrees) each wheel should rotate
     */
    private int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
     * Computes the angle each wheel should rotate
     * in order to rotate in place a certain angle
     *
     * @param radius        radius in centimeters
     * @param angle         angle in radians
     *
     * @return angle (in degrees) each wheel should rotate
     */
    private int convertAngle(double radius, double angle) {
        return convertDistance(radius, robot.width * angle / 2.0);
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
        UltrasonicSensor sonic;
        if (side == RIGHT)
            sonic = robot.rightSonic;
        else if (side == LEFT)
            sonic = robot.leftSonic;
        else
            sonic = robot.centerSonic;

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

        // // GET LIGHT VALUE
        // intensity[side][lightCounter[side]++] = color.getRawLightValue();

        // if (lightCounter[side] == intensity[side].length)
        //     lightCounter[side] = 0;

        // for (int i = 0; i < intensity[side].length; i++)
        //     sortedIntensity[i] = intensity[side][i];

        // Arrays.sort(sortedIntensity);

        // // COMPUTE AVERAGE MEDIAN
        // int sum = 0;

        // for (int i = 3; i < sortedIntensity.length - 3; i++)
        //     sum += sortedIntensity[i];

        // int median = sum / (sortedIntensity.length - 6);

        // LCD.drawString(format(median), 0, 6 + side);

        // // ANALYZE
        // if (median <= threshold[side]) {
        //     for (int i = 0; i < intensity[side].length; i++)
        //         intensity[side][i] = 600;

        //     return true;
        // }

        if (color.getRawLightValue() < threshold[side])
            return true;

        return false;
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
