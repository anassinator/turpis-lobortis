import lejos.nxt.*;
import lejos.util.*;

/**
 * Navigates through the game
 * @author Anass Al-Wohoush, Mohamed Kleit
 * @version 2.0
 */
public class Navigation {
    // OBJECTS
    public static Robot robot;
    public static Odometer odometer;
    public static Recognition recognizer;
    public static Localizer localizer;

    // VARIABLES
    public static double[] homeZone, enemyZone, homeTarget, enemyTarget;
    public static boolean turning = false;
    public static int HOME_FLAG, ENEMY_FLAG;
    private int[][] distance = new int[3][3];
    private int[] sonicCounter = {0,0,0};
    public static double[] TARGET = {0.00, 0.00};

    // DEFINES
    public static final double SIZE_OF_TILE = 30.48;
    public static final double SIZE_OF_FIELD = 12;
    private static final int ROTATE_SPEED = 50;
    private static final int LEFT = 1, FRONT = 2, RIGHT = 3;
    private static final int BANDWIDTH = 3, BANDCENTRE = 18;
    private static final int LOW = 180, MEDIUM = 240, HIGH = 360;
    public static int HIGH_SPEED = 265, MOTOR_SPEED = 175, MED_SPEED = 125, SLOW_SPEED = 100;

    /**
     * Navigation constructor
     *
     * @param robot         robot object containing robot's dimensions and motor ports
     * @param odometer      odometer object containing robot's position and orientation
     * @param info          information containing opponent's flag location and drop off zone
     */
    public Navigation(Robot robot, Odometer odometer, int[] info) {
        this.robot = robot;
        this.odometer = odometer;

        // PARSE
        homeZone    = new double[]  {  SIZE_OF_TILE * info[1],     SIZE_OF_TILE * info[2],
                                       SIZE_OF_TILE * info[3],     SIZE_OF_TILE * info[4]  };
        enemyZone   = new double[]  {  SIZE_OF_TILE * info[5],     SIZE_OF_TILE * info[6],
                                       SIZE_OF_TILE * info[7],     SIZE_OF_TILE * info[8]  };

        homeTarget  = new double[]  {  SIZE_OF_TILE * info[9],     SIZE_OF_TILE * info[10] };
        enemyTarget = new double[]  {  SIZE_OF_TILE * info[11],    SIZE_OF_TILE * info[12] };

        HOME_FLAG   = info[13];
        ENEMY_FLAG  = info[14];

        recognizer  = new Recognition(robot);

        // SET UP MOVING AVERAGE FILTERS
        for (int i = 0; i < distance[0].length; i++)
            distance[LEFT - 1][i] = robot.leftSonic.getDistance();

        for (int i = 0; i < distance[0].length; i++)
            distance[FRONT - 1][i] = robot.centerSonic.getDistance();

        for (int i = 0; i < distance[0].length; i++)
            distance[RIGHT - 1][i] = robot.rightSonic.getDistance();
    }

    /**
     * Navigates to opponent's flag, captures it and drops it
     * off while avoiding obstacles
     */
    public void run() {
        setMotorSpeeds(MEDIUM, MEDIUM);

        approach();

        search();
    }

    /**
     * Search for opponent's flag
     */
    public void search() {
        boolean done = false;
        int detected;
        robot.avoidPlz = false;

        while (!done) {
            // GO FORWARD SLOWLY UNTIL AN OBSTACLE IS DETECTED
            robot.leftMotor.forward();
            robot.rightMotor.forward();
            robot.leftMotor.setSpeed(100);
            robot.rightMotor.setSpeed(100);

            // WAIT UNTIL OBJECT DETECTED
            while ((detected = detect()) <= 0) {
                // IF WALL AHEAD, ROTATE AND TRY AGAIN
                if (detected == -1) {
                    stop();
                    turn(-Math.PI / 2);
                    goForward(SIZE_OF_TILE);
                    turn(-Math.PI / 2);
                    robot.leftMotor.forward();
                    robot.rightMotor.forward();
                    robot.leftMotor.setSpeed(100);
                    robot.rightMotor.setSpeed(100);
                }
            }

            // ROTATE TOWARD DETECTED OBJECT
            stop();
            switch (detected) {
                case LEFT:
                    robot.leftMotor.rotate(-convertAngle(robot.leftRadius, Math.toRadians(45)), true);
                    robot.rightMotor.rotate(convertAngle(robot.rightRadius, Math.toRadians(45)), false);
                    stop();
                    break;

                case FRONT:
                    break;

                case RIGHT:
                    robot.leftMotor.rotate(convertAngle(robot.leftRadius, Math.toRadians(45)), true);
                    robot.rightMotor.rotate(-convertAngle(robot.rightRadius, Math.toRadians(45)), false);
                    stop();
                    break;
            }

            // GRAB
            goBackward(30);
            robot.claw.drop();
            goForward(30);
            robot.claw.grab();

            // RECOGNIZE
            int recognized = recognizer.recognize();
            int counter = 0;
            while (counter++ < 3 && recognized == recognizer.IDK) {
                recognized = recognizer.recognize();
            }
            if (recognized == recognizer.WOOD) {
                robot.claw.drop();
                goForward(10);
                robot.claw.grab();
                recognized = recognizer.recognize();
            }
            if (recognized != ENEMY_FLAG) {
                turn(-Math.PI);
                goForward(20);
                robot.claw.drop();
                goBackward(20);
                robot.claw.grab();
                turnTo(Math.PI / 2);
            } else {
                Sound.systemSound(false, 3);
                break;
            }
        }
    }

    /**
     * Navigates to opponent's zone from the best angle
     */
    public void approach() {
        // GET CURRENT COORDINATES
        double[] HERE = { odometer.getX(), odometer.getY() };

        // GET ZONE'S CORNERS
        double[] LOWER_LEFT  = { enemyZone[0], enemyZone[1] };
        double[] LOWER_RIGHT = { enemyZone[2], enemyZone[1] };
        double[] UPPER_LEFT  = { enemyZone[0], enemyZone[3] };
        double[] UPPER_RIGHT = { enemyZone[2], enemyZone[3] };

        // GET ZONE'S SIDES
        double[][] NORTH = { UPPER_LEFT,  UPPER_RIGHT };
        double[][] EAST  = { LOWER_RIGHT, UPPER_RIGHT };
        double[][] SOUTH = { LOWER_LEFT,  LOWER_RIGHT };
        double[][] WEST  = { LOWER_LEFT,  UPPER_LEFT  };

        // CHECK IF SIDE IS ON EDGE OF BOARD
        boolean northIsEdge = (UPPER_LEFT[1] == (SIZE_OF_FIELD - 1) * SIZE_OF_TILE);
        boolean westIsEdge = (UPPER_LEFT[0] == -1 * SIZE_OF_TILE);
        boolean southIsEdge = (LOWER_LEFT[1] == -1 * SIZE_OF_TILE);
        boolean eastIsEdge = (UPPER_RIGHT[0] == (SIZE_OF_FIELD - 1) * SIZE_OF_TILE);

        // SELECT CLOSEST SHORT SIDE
        double[][] CLOSEST;
        double x, y, theta;
        if (distance(LOWER_LEFT, LOWER_RIGHT) < distance(LOWER_LEFT, UPPER_LEFT)) {
            if (southIsEdge) {
                CLOSEST = NORTH;
                theta = 3 * Math.PI / 2;
            } else if (northIsEdge) {
                CLOSEST = SOUTH;
                theta = Math.PI / 2;
            } else {
                if (distance(HERE, LOWER_LEFT) < distance(HERE, UPPER_LEFT)) {
                    CLOSEST = SOUTH;
                    theta = Math.PI / 2;
                } else {
                    CLOSEST = NORTH;
                    theta = 3 * Math.PI / 2;
                }
            }
        } else {
            if (westIsEdge) {
                CLOSEST = EAST;
                theta = Math.PI;
            } else if (eastIsEdge) {
                CLOSEST = WEST;
                theta = 0;
            } else {
                if (distance(HERE, LOWER_LEFT) < distance(HERE, LOWER_RIGHT)) {
                    CLOSEST = WEST;
                    theta = 0;
                } else {
                    CLOSEST = EAST;
                    theta = Math.PI;
                }
            }
        }

        // CALCULATE POINT TO GO TO
        x = (int)(((CLOSEST[0][0] + CLOSEST[1][0]) / 2) / SIZE_OF_TILE) * SIZE_OF_TILE;
        y = (int)(((CLOSEST[0][1] + CLOSEST[1][1]) / 2) / SIZE_OF_TILE) * SIZE_OF_TILE;

        // GO TO ZONE
        goTo(x, y);
        turnTo(theta);
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
     * Travels to coordinates x and y while avoiding and relocalizes
     *
     * @param x             x coordinate in centimeters
     * @param y             y coordinate in centimeters
     */
    public void goTo(double x, double y) {
        // AVOID
        robot.avoidPlz = true;
        double[] WAYPOINT = {odometer.getX(), odometer.getY()};
        int DIVISION = 4;

        // DIVIDE PATH
        double distanceToTravel = Math.sqrt(x * x + y * y);
        int numberOfWaypoints = (int)(distanceToTravel / (DIVISION * SIZE_OF_TILE));
        for (int i = 0; i < numberOfWaypoints - 1; i++) {
            int QUADRANT = 1;
            int[] DIRECTION = {1, 1};

            // FIGURE OUT DIRECTION
            double angle = Math.atan2(y - odometer.getY(), x - odometer.getX());
            if (angle >= 0 && angle <= Math.PI / 2) {
                QUADRANT = 1;
            } else if (angle >= Math.PI / 2 && angle <= Math.PI) {
                QUADRANT = 2;
                DIRECTION[0] = -1;
            } else if (angle >= Math.PI && angle <= 3 * Math.PI / 2) {
                QUADRANT = 3;
                DIRECTION[0] = -1;
                DIRECTION[1] = -1;
            } else {
                QUADRANT = 4;
                DIRECTION[1] = -1;
            }

            // COMPUTE WAYPOINT
            WAYPOINT[0] += DIRECTION[0] * Math.ceil(DIVISION * Math.cos(angle)) * SIZE_OF_TILE;
            WAYPOINT[1] += DIRECTION[1] * Math.ceil(DIVISION * Math.sin(angle)) * SIZE_OF_TILE;

            // MAKE SURE NOT ON CRACK
            int crack = isOnCrack(WAYPOINT[0], WAYPOINT[1]);
            if (crack == 0) {           // CRACK ON X
                WAYPOINT[0] += DIRECTION[0] * SIZE_OF_TILE;
            } else if (crack == 1) {    // CRACK ON Y
                WAYPOINT[1] += DIRECTION[1] * SIZE_OF_TILE;
            } else if (crack == 2) {    // CRACK ON X AND Y
                WAYPOINT[0] += DIRECTION[0] * SIZE_OF_TILE;
                WAYPOINT[1] += DIRECTION[1] * SIZE_OF_TILE;
            }

            // TRAVEL AND CHECK IF NOT GOING TOWARD OBSTACLE
            if (!travelTo(WAYPOINT[0], WAYPOINT[1])) {
                if (QUADRANT == 1)
                    travelTo(WAYPOINT[0] + DIRECTION[0] * SIZE_OF_TILE,
                             WAYPOINT[1] + DIRECTION[1] * SIZE_OF_TILE);
                else if (QUADRANT == 2)
                    travelTo(WAYPOINT[0] - DIRECTION[0] * SIZE_OF_TILE,
                             WAYPOINT[1] + DIRECTION[1] * SIZE_OF_TILE);
                else if (QUADRANT == 3)
                    travelTo(WAYPOINT[0] - DIRECTION[0] * SIZE_OF_TILE,
                             WAYPOINT[1] - DIRECTION[1] * SIZE_OF_TILE);
                else if (QUADRANT == 4)
                    travelTo(WAYPOINT[0] + DIRECTION[0] * SIZE_OF_TILE,
                             WAYPOINT[1] - DIRECTION[1] * SIZE_OF_TILE);               
            }

            // RELOCALIZE
            localizer.relocalize();
        }

        // TRAVEL TO FINAL DESTINATION AND RELOCALIZE
        travelTo(x, y);
        localizer.relocalize();
        travelTo(x, y);
    }

    /**
     * Travels forward a given distance
     *
     * @param distance     distance in centimeters
     */
    public void goForward(double distance) {
        setMotorSpeeds(MEDIUM, MEDIUM);
        robot.leftMotor.rotate(convertDistance(robot.leftRadius, distance), true);
        robot.rightMotor.rotate(convertDistance(robot.rightRadius, distance), false);
    }

    /**
     * Travels backward a given distance
     *
     * @param distance     distance in centimeters
     */
    public void goBackward(double distance) {
        setMotorSpeeds(MEDIUM, MEDIUM);
        robot.leftMotor.rotate(-convertDistance(robot.leftRadius, distance), true);
        robot.rightMotor.rotate(-convertDistance(robot.rightRadius, distance), false);
    }

    /**
     * Travels to coordinates x and y
     *
     * @param x             x coordinate in centimeters
     * @param y             y coordinate in centimeters
     *
     * @return <code>true</code> if travelled to point successfully; <code>false</code> otherwise
     */
    public boolean travelTo(double x, double y) {
        // SET SPEEDS
        setMotorSpeeds(MEDIUM, MEDIUM);

        // SET TARGET
        TARGET[0] = x;
        TARGET[1] = y;

        // CALCULATE DESIRED LOCATION FROM CURRENT LOCATION
        // BY MEASURING THE DIFFERENCE
        double xToTravel = x - odometer.getX();
        double yToTravel = y - odometer.getY();

        double angle = Math.atan2(yToTravel, xToTravel);

        // MEASURE DESIRED ORIENTATION BY TRIGONOMETRY
        // ATAN2 DEALS WITH CORRECT SIGNS FOR US
        double desiredOrientation = Math.atan2(yToTravel, xToTravel);

        // SPIN TO DESIRED ANGLE
        turnTo(desiredOrientation);

        // MEASURE DESIRED DISTANCE BY PYTHAGORUS
        double desiredDistance = Math.sqrt(xToTravel * xToTravel + yToTravel * yToTravel);

        // MOVE FORWARD DESIRED DISTANCE AND RETURN IMMEDIATELY
        robot.leftMotor.rotate(convertDistance(robot.leftRadius, desiredDistance), true);
        robot.rightMotor.rotate(convertDistance(robot.rightRadius, desiredDistance), robot.avoidPlz);

        // KEEP MOVING AND STOP IF OBSTACLE
        boolean avoided = false;
        while (robot.avoidPlz && isNavigating()) {
            avoided = avoid(detect());
        }

        // CHECK WHICH QUADRANT WE'RE FACING
        int QUADRANT = 0;
        if (angle >= 0 && angle <= Math.PI / 2)
            QUADRANT = 1;
        else if (angle >= Math.PI / 2 && angle <= Math.PI)
            QUADRANT = 2;
        else if (angle >= Math.PI && angle <= 3 * Math.PI / 2)
            QUADRANT = 3;
        else
            QUADRANT = 4;

        // CHECK IF DESTINATION IS BEHIND US
        boolean destinationIsBehindUs = false;
        if (QUADRANT == 1)
            destinationIsBehindUs = odometer.getX() > x || odometer.getY() > y;
        else if (QUADRANT == 2)
            destinationIsBehindUs = odometer.getX() < x || odometer.getY() > y;
        else if (QUADRANT == 3)
            destinationIsBehindUs = odometer.getX() < x || odometer.getY() < y;
        else
            destinationIsBehindUs = odometer.getX() > x || odometer.getY() < y;

        // RETURN UNSUCCESSFUL
        if (avoided && destinationIsBehindUs)
            return false;

        // REPEAT IF UNACCEPTABLE
        if (!goodEnough(x,y))
            travelTo(x,y);

        // STOP MOTORS
        stop();

        // RETURN SUCCESFUL
        return true;
    }

    /**
     * Returns whether the robot is close enough to the desired coordinates
     *
     * @param x             desired x value
     * @param y             desired y value
     *
     * @return <code>true</code> if robot is close enough; <code>false</code> otherwise
     */
    public boolean goodEnough(double x, double y) {
        return Math.abs(x - odometer.getX()) <= 1.00 && Math.abs(y - odometer.getY()) <= 1.00;
    }

    /**
     * Returns whether the robot is close enough to the desired orientation
     *
     * @param theta         desired orientation
     *
     * @return <code>true</code> if robot is close enough; <code>false</code> otherwise
     */
    public boolean goodEnough(double theta) {
        return Math.abs(theta - odometer.getTheta()) <= Math.toDegrees(1.00);
    }


    /**
     * Returns whether the robot is navigating
     *
     * @return <code>true</code> if robot is navigating; <code>false</code> otherwise
     */
    public boolean isNavigating() {
        return robot.leftMotor.isMoving() || robot.rightMotor.isMoving();
    }

    /**
     * Returns whether there is an object within range of the ultrasonic sensors
     * and where
     *
     * <TABLE BORDER=1>
     * <TR><TH>Obstacle Code</TH><TH>Wooden Block</TH></TR>
     * <TR><TD>0</TD><TD>No obstacle detected</TD></TR>
     * <TR><TD>1</TD><TD>Obstacle to the left</TD></TR>
     * <TR><TD>2</TD><TD>Obstacle straight ahead</TD></TR>
     * <TR><TD>3</TD><TD>Obstacle to the right</TD></TR>
     * </TABLE>
     *
     * @return the obstacle code
     */
    public int detect() {
        boolean obstacleLeft = isWall(LEFT);
        boolean obstacleAhead = !robot.claw.isDown && isWall(FRONT);
        boolean obstacleRight = isWall(RIGHT);

        if (inFrontOfWall()) {
            return -1;
        } else if (nextToLeftWall()) {
            if (obstacleAhead) {
                return FRONT;
            } else if (obstacleRight) {
                return RIGHT;
            }
        } else if (nextToRightWall()) {
            if (obstacleAhead) {
                return FRONT;
            } else if (obstacleLeft) {
                return LEFT;
            }
        } else {
            if (obstacleAhead) {
                return FRONT;
            } else if (obstacleRight) {
                return RIGHT;
            } else if (obstacleLeft) {
                return LEFT;
            }
        }
        return 0;
    }

    /**
     * Detects whether it is currently detecting the left wall
     *
     * @return true if sees left wall
     */
    private boolean nextToLeftWall() {
        double xPosition = odometer.getX();
        double yPosition = odometer.getY();
        double theta = odometer.getTheta();

        boolean up = (xPosition < 0.5 * SIZE_OF_TILE
                      && theta < (Math.toRadians(100))
                      && theta > (Math.toRadians(80)));

        boolean right = (yPosition > (SIZE_OF_FIELD - 2.5) * SIZE_OF_TILE
                         && (theta < (Math.toRadians(10))
                             || theta > (Math.toRadians(350))));

        boolean down = (xPosition > (SIZE_OF_FIELD - 2.5) * SIZE_OF_TILE
                        && theta > (Math.toRadians(260))
                        && theta < (Math.toRadians(280)));

        boolean left = (yPosition < 0.5 * SIZE_OF_TILE
                        && theta < (Math.toRadians(190))
                        && theta > (Math.toRadians(170)));

        return (up || down || left || right);
    }

    /**
     * Detects whether it is currently detecting the right wall
     *
     * @return true if sees right wall
     */
    private boolean inFrontOfWall() {
        double xPosition = odometer.getX();
        double yPosition = odometer.getY();
        double theta = odometer.getTheta();

        boolean left = (xPosition < 0.5 * SIZE_OF_TILE
                        && theta < (Math.toRadians(190))
                        && theta > (Math.toRadians(170)));

        boolean right = (xPosition > (SIZE_OF_FIELD - 2.5) * SIZE_OF_TILE
                         && (theta < (Math.toRadians(10))
                             || theta > (Math.toRadians(350))));

        boolean up = (yPosition > (SIZE_OF_FIELD - 2.5) * SIZE_OF_TILE
                      && theta < (Math.toRadians(100))
                      && theta > (Math.toRadians(80)));

        boolean down = (yPosition < 0.5 * SIZE_OF_TILE
                      && theta < (Math.toRadians(280))
                      && theta > (Math.toRadians(260)));

        return (up || down || left || right);
    }

    /**
     * Detects whether it is currently detecting the right wall
     *
     * @return true if sees right wall
     */
    private boolean nextToRightWall() {
        double xPosition = odometer.getX();
        double yPosition = odometer.getY();
        double theta = odometer.getTheta();

        boolean down = (xPosition < 0.5 * SIZE_OF_TILE
                        && theta > (Math.toRadians(260))
                        && theta < (Math.toRadians(280)));

        boolean left = (yPosition > (SIZE_OF_FIELD - 2.5) * SIZE_OF_TILE
                        && theta < (Math.toRadians(190))
                        && theta > (Math.toRadians(170)));

        boolean up = (xPosition > (SIZE_OF_FIELD - 2.5) * SIZE_OF_TILE
                      && theta < (Math.toRadians(100))
                      && theta > (Math.toRadians(80)));

        boolean right = (yPosition < 0.5 * SIZE_OF_TILE
                         && (theta < (Math.toRadians(10))
                             || theta > (Math.toRadians(350))));

        return (up || down || left || right);
    }

    /**
     * Computes the minimal angle the robot should turn by
     * according to the current theta read on the odometer
     *
     * @param radians      angle in radians
     *
     * @return the minimal angle in radians
     */
    public double getMinimalAngle(double radians) {
        if (radians > Math.PI)
            return getMinimalAngle(radians - Math.PI * 2);
        else if (radians < -Math.PI)
            return getMinimalAngle(radians + Math.PI * 2);
        return radians;
    }

    /**
     * Sets motor speeds
     *
     * @param leftSpeed     speed of left motor in degrees/second
     * @param rightSpeed    speed of right motor in degrees/second
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
        robot.leftMotor.setSpeed((float)leftSpeed);
        robot.rightMotor.setSpeed((float)rightSpeed);
    }

    /**
     * Sets rotate speed
     *
     * @param rotateSpeed   speed of motor in degrees/second
     */
    public void setMotorRotateSpeed(double rotateSpeed) {
        setMotorSpeeds(rotateSpeed, -rotateSpeed);
    }

    /**
     * Turns to the angle computed in getMinimalAngle method
     *
     * @param theta         orientation in radians
     */
    public void turnTo(double theta) {
        setMotorSpeeds(LOW, LOW);
        // CALCULATE ANGLE TO ROTATE REALTIVE TO CURRENT ANGLE
        double currentOrientation = odometer.getTheta();
        double angle = theta - currentOrientation;

        // CORRECT ANGLE TO REMAIN WITHIN -180 AND 180 DEGREES
        // TO MINIMIZE ANGLE TO SPIN
        if (angle < -3.14)
            angle += 6.28;
        else if (angle > 3.14)
            angle -= 6.28;

        // SET TURNING FLAG
        turning = true;

        // ROTATE SAID ANGLE AND WAIT UNTIL DONE
        robot.leftMotor.rotate(-convertAngle(robot.leftTurningRadius, angle), true);
        robot.rightMotor.rotate(convertAngle(robot.rightTurningRadius, angle), false);

        // STOP
        stop();

        // CHECK IF GOOD ENOUGH
        // if (!goodEnough(theta))
        //     turnTo(theta);

        // RESET TURNING FLAG
        turning = false;
        setMotorSpeeds(MEDIUM, MEDIUM);
    }

   /**
     * Turns the angle specified
     *
     * @param theta         orientation in radians
     */
    public void turn(double theta) {
        setMotorSpeeds(LOW, LOW);
        // CORRECT ANGLE TO REMAIN WITHIN -180 AND 180 DEGREES
        // TO MINIMIZE ANGLE TO SPIN
        if (theta < -3.14)
            theta += 6.28;
        else if (theta > 3.14)
            theta -= 6.28;

        // SET TURNING FLAG
        turning = true;

        // ROTATE SAID ANGLE AND WAIT UNTIL DONE
        robot.leftMotor.rotate(-convertAngle(robot.leftTurningRadius, theta), true);
        robot.rightMotor.rotate(convertAngle(robot.rightTurningRadius, theta), false);

        stop();

        // RESET TURNING FLAG
        turning = false;

        setMotorSpeeds(MEDIUM, MEDIUM);
    }

    /**
     * Stops the robot
     */
    public void stop() {
        robot.leftMotor.stop(true);
        robot.rightMotor.stop(false);
    }

    /**
     * Avoids obstacle depending on where it has been detected
     *
     * <TABLE BORDER=1>
     * <TR><TH>Obstacle Code</TH><TH>Wooden Block</TH></TR>
     * <TR><TD>0</TD><TD>No obstacle detected</TD></TR>
     * <TR><TD>1</TD><TD>Obstacle to the left</TD></TR>
     * <TR><TD>2</TD><TD>Obstacle straight ahead</TD></TR>
     * <TR><TD>3</TD><TD>Obstacle to the right</TD></TR>
     * </TABLE>
     *
     * @return <code>true</code> if obstacle avoided; <code>false</code> otherwise
     */
    public boolean avoid(int direction) {
        // RETURN IF NO OBSTACLE
        if (direction <= 0)
            return false;

        // STOP AND STEP BACK
        stop();
        goBackward(5);

        // IF FRONT SELECT OPTIMAL SIDE
        if (direction == FRONT) {
            // GET DELTA THETA
            double theta = odometer.getTheta();
            double deltaTheta = Math.atan2(TARGET[1], TARGET[0]) - theta;

            // SET DIRECTION
            if (deltaTheta > 0)
                direction = RIGHT;
            else
                direction = LEFT;
        }

        if (direction == LEFT || direction == FRONT) {
            boolean wall = true;
            double exitAngle;

            int error;
            turn(-Math.PI / 2);
            exitAngle = (odometer.getTheta() + Math.PI) % (2 * Math.PI);

            do {
                int dist = (int) (robot.leftSonic.getDistance()
                         * (Math.cos(Math.PI / 4)));
                robot.leftMotor.forward();
                robot.rightMotor.forward();

                error = BANDCENTRE - dist;
                // WITHIN ACCEPTABLE DISTANCE FROM WALL
                if (Math.abs(error) <= BANDWIDTH) {
                    // NO CHANGE IN EITHER MOTOR SPEEDS
                    robot.leftMotor.setSpeed(MOTOR_SPEED);
                    robot.rightMotor.setSpeed(MOTOR_SPEED);
                }
                // WITHIN BANDWIDTH + 2, SLIGHT TURN OUTWARD
                else if (error <= (BANDWIDTH + 5) && error > 0) {
                    robot.leftMotor.setSpeed(HIGH_SPEED);
                    robot.rightMotor.setSpeed(175);
                }
                // WITHIN BANDWIDTH + 3 SHARPER TURN
                else if (error > (BANDWIDTH + 8)) {
                    robot.leftMotor.setSpeed(HIGH_SPEED);
                    robot.rightMotor.setSpeed(SLOW_SPEED);
                }
                // TOO FAR TO WALL
                else if (error <= -(BANDWIDTH + 5) && error > -8) {
                    // DECREASE SPEED OF INNER WHEEL, ERROR TERM NEEDED TO
                    // COMPENSATE FOR SCANNER READINGS OF 255
                    robot.leftMotor.setSpeed(SLOW_SPEED);
                    // INCREASE SPEED OF OUTER WHEEL
                    robot.rightMotor.setSpeed(175);
                } else {
                    // INCREASE SPEED OF INNER WHEEL
                    robot.leftMotor.setSpeed(SLOW_SPEED + 25);
                    // DECREASE SPEED OF OUTER WHEEL
                    robot.rightMotor.setSpeed(HIGH_SPEED);
                }

                if (odometer.getTheta() > exitAngle - Math.toRadians(10)
                    && odometer.getTheta() < exitAngle) {
                    Sound.beep();
                    wall = false;
                }
            } while (wall);
            goForward(10);
            turn(-Math.PI / 2);
        } else if (direction == RIGHT) {
            boolean wall = true;
            double exitAngle;

            int error;
            turn(Math.PI / 2);
            exitAngle = (odometer.getTheta() + Math.PI) % (2 * Math.PI);

            do {
                int dist = (int) (robot.rightSonic.getDistance()
                         * (Math.cos(Math.PI / 4)));
                robot.leftMotor.forward();
                robot.rightMotor.forward();

                error = BANDCENTRE - dist;

                // WITHIN ACCEPTABLE DISTANCE FROM WALL
                if (Math.abs(error) <= BANDWIDTH) {
                    // NO CHANGE IN EITHER MOTOR SPEEDS
                    robot.leftMotor.setSpeed(MOTOR_SPEED);
                    robot.rightMotor.setSpeed(MOTOR_SPEED);
                }

                // WITHIN BANDWIDTH + 2, SLIGHT TURN INWARD
                else if (error <= (BANDWIDTH + 5) && error > 0) {
                    robot.leftMotor.setSpeed(175);
                    robot.rightMotor.setSpeed(HIGH_SPEED);
                }

                // WITHIN BANDWIDTH + 3 SHARPER TURN
                else if (error > (BANDWIDTH + 8)) {
                    robot.leftMotor.setSpeed(SLOW_SPEED);
                    robot.rightMotor.setSpeed(HIGH_SPEED);
                }

                // TOO FAR TO WALL
                else if (error <= -(BANDWIDTH + 5) && error > -8) {
                    // DECREASE SPEED OF OUTER WHEEL, ERROR TERM NEEDED TO
                    // COMPENSATE FOR SCANNER READINGS OF 255
                    robot.leftMotor.setSpeed(175);
                    // INCREASE SPEED OF OUTER WHEEL
                    robot.rightMotor.setSpeed(SLOW_SPEED);
                } else {
                    // INCREASE SPEED OF OUTER WHEEL
                    robot.leftMotor.setSpeed(HIGH_SPEED);
                    // DECREASE SPEED OF INNER WHEEL
                    robot.rightMotor.setSpeed(SLOW_SPEED + 25);
                }

                // CHECK EXIT ANGLE
                if (odometer.getTheta() > exitAngle - Math.toRadians(10)
                    && odometer.getTheta() < exitAngle) {
                    Sound.beep();
                    wall = false;
                }
            } while (wall);
            goForward(10);
            turn(Math.PI / 2);
        }

        return true;
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
    private static int convertDistance(double radius, double distance) {
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
    private static int convertAngle(double radius, double angle) {
        return convertDistance(radius, robot.width * angle / 2.0);
    }

    /**
     * Returns whether a wall is detected by the selected ultrasonic sensor
     *
     * @param side         select ultrasonic sensor, 1 for LEFT, 2 for FRONT and 3 for RIGHT
     *
     * @return <code>true</code> if wall detected; <code>false</code> otherwise
     */
    private boolean isWall(int side) {
        // SELECT CORRECT ULTRASONIC SENSOR AND THRESHOLD
        UltrasonicSensor sonic = null;
        int threshold = BANDCENTRE;
        switch (side) {
            case LEFT:
                sonic = robot.leftSonic;
                threshold *= Math.cos(Math.PI / 4);
                break;
            case FRONT:
                sonic = robot.centerSonic;
                break;
            case RIGHT:
                sonic = robot.rightSonic;
                threshold *= Math.cos(Math.PI / 4);
                break;
        }

        // GET DISTANCE
        distance[side - 1][sonicCounter[side - 1]++] = sonic.getDistance();

        if (sonicCounter[side - 1] == distance[0].length)
            sonicCounter[side - 1] = 0;

        // COMPUTE AVERAGE
        int sum = 0;

        for (int i = 0; i < distance[side - 1].length; i++)
            sum += distance[side - 1][i];

        int avg = sum / distance[side - 1].length;

        // ANALYZE
        if (avg < threshold) {
            return true;
        } else
            return false;
    }

    /**
     * Returns whether a coordinate is on a crack or not
     *
     * @param x             x coordinate
     * @param y             y coordinate
     *
     * <TABLE BORDER=1>
     * <TR><TH>Return Code</TH><TH>Description</TH></TR>
     * <TR><TD>-1</TD><TD>Not on crack</TD></TR>
     * <TR><TD>0</TD><TD>X coordinate is on crack</TD></TR>
     * <TR><TD>1</TD><TD>Y coordinate is on crack</TD></TR>
     * <TR><TD>2</TD><TD>Both coordinates are on crack</TD></TR>
     * </TABLE>
     *
     * @return the return code
     */
    private int isOnCrack(double x, double y) {
        boolean xOnCrack = x == 3 * SIZE_OF_TILE || x == 7 * SIZE_OF_TILE;
        boolean yOnCrack = y == 3 * SIZE_OF_TILE || y == 7 * SIZE_OF_TILE;
        boolean bothOnCrack = xOnCrack && yOnCrack;

        if (bothOnCrack)
            return 2;
        else if (xOnCrack)
            return 0;
        else if (yOnCrack)
            return 1;
        else
            return -1;
    }
}
