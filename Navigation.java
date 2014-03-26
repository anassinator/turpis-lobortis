import lejos.nxt.*;
import lejos.util.*;

/**
 * Navigates through the game
 * @author Anass Al-Wohoush, Mohamed Kleit
 * @version 1.2
 */
public class Navigation {
    // OBJECTS
    public static Robot robot;
    public static Odometer odometer;
    public static Recognition recognizer;

    // VARIABLES
    public static int[] courseInfo;
    public static boolean turning = false;

    // DEFINES
    private static final int ROTATE_SPEED = 50;
    private static final int LEFT = 1, CENTER = 2, RIGHT = 3;
    private static final int BANDWIDTH = 3, BANDCENTRE = 15;
    private static final int LOW = 180, HIGH = 360;

    // FLAG TO LOOK FOR
    private static int FLAG = 3;

    /**
     * Navigation constructor
     *
     * @param robot         robot object containing robot's dimensions and motor ports
     * @param odometer      odometer object containing robot's position and orientation
     * @param courseInfo    information containing opponent's flag location and drop off zone
     */
    public Navigation(Robot robot, Odometer odometer, int[] courseInfo) {
        this.robot = robot;
        this.odometer = odometer;
        this.courseInfo = courseInfo;
        recognizer = new Recognition(robot);
    }

    /**
     * Navigates to opponent's flag, captures it and drops it
     * off while avoiding obstacles
     */
    public void run() {
        // ...
        double side = 30.48 * 3;
        travelTo(0, side);
        travelTo(side, side);
        travelTo(side, 0);
        travelTo(0, 0);
    }

    /**
     * Search for opponent's flag
     */
    public void search() {
        boolean done = false;

        while (!done) {
            // GO FORWARD SLOWLY UNTIL AN OBSTACLE IS DETECTED
            robot.leftMotor.setSpeed(LOW);
            robot.rightMotor.setSpeed(LOW);

            while (detect() == 0);

            stop();

            int detected = detect();

            // ROTATE TOWARD DETECTED OBJECT
            switch (detected) {
                case LEFT:
                    robot.leftMotor.rotate(-convertAngle(robot.leftRadius, 60), true);
                    robot.rightMotor.rotate(convertAngle(robot.rightRadius, 60), true);
                    while (detect() != CENTER);
                    stop();
                    break;

                case CENTER:
                    break;

                case RIGHT:
                    robot.leftMotor.rotate(convertAngle(robot.leftRadius, 60), true);
                    robot.rightMotor.rotate(-convertAngle(robot.rightRadius, 60), true);
                    while (detect() != CENTER);
                    stop();
                    break;
            }

            // GRAB
            goBackward(10);
            robot.claw.drop();
            goForward(20);
            robot.claw.grab();

            // RECOGNIZE
            if (recognizer.recognize() != FLAG)
                robot.claw.drop();
            else
                break;
        }
    }

    /**
     * Travels forward a given distance
     *
     * @param distance     distance in centimeters
     */
    public void goForward(double distance) {
        robot.leftMotor.rotate(convertDistance(robot.leftRadius, distance), true);
        robot.rightMotor.rotate(convertDistance(robot.rightRadius, distance), false);
    }

    /**
     * Travels backward a given distance
     *
     * @param distance     distance in centimeters
     */
    public void goBackward(double distance) {
        robot.leftMotor.rotate(-convertDistance(robot.leftRadius, distance), true);
        robot.rightMotor.rotate(-convertDistance(robot.rightRadius, distance), false);
    }

    /**
     * Travels to coordinates x and y
     *
     * @param x             x coordinate in centimeters
     * @param y             y coordinate in centimeters
     */
    public void travelTo(double x, double y) {
        // CALCULATE DESIRED LOCATION FROM CURRENT LOCATION
        // BY MEASURING THE DIFFERENCE
        double xToTravel = x - odometer.getX();
        double yToTravel = y - odometer.getY();

        // MEASURE DESIRED ORIENTATION BY TRIGONOMETRY
        // ATAN2 DEALS WITH CORRECT SIGNS FOR US
        double desiredOrientation = Math.atan2(yToTravel, xToTravel);

        // SPIN TO DESIRED ANGLE
        turnTo(desiredOrientation);

        // MEASURE DESIRED DISTANCE BY PYTHAGORUS
        double desiredDistance = Math.sqrt(xToTravel * xToTravel + yToTravel * yToTravel);

        // MOVE FORWARD DESIRED DISTANCE AND RETURN IMMEDIATELY
        robot.leftMotor.rotate(convertDistance(robot.leftRadius, desiredDistance), true);
        robot.rightMotor.rotate(convertDistance(robot.rightRadius, desiredDistance), true);

        // KEEP MOVING AND STOP IF OBSTACLE
        while (isNavigating()) {
            avoid(detect());
        }

        // REPEAT IF UNACCEPTABLE
        if (!goodEnough(x,y))
            travelTo(x,y);

        // STOP MOTORS
        stop();
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
        return Math.abs(theta - odometer.getTheta()) <= 0.05;
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
        boolean obstacleLeft = robot.leftSonic.getDistance() <= 15;
        boolean obstacleAhead = !robot.claw.isDown && robot.centerSonic.getDistance() <= 15;
        boolean obstacleRight = robot.rightSonic.getDistance() <= 15;

        if (obstacleLeft)
            return LEFT;
        else if (obstacleRight)
            return RIGHT;
        else if (obstacleAhead)
            return CENTER;
        else
            return 0;
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
        robot.leftMotor.setSpeed((int) leftSpeed);
        robot.rightMotor.setSpeed((int) rightSpeed);
        if (leftSpeed > 0) {
            robot.leftMotor.forward();
        }
        else {
            robot.leftMotor.backward();
        }
        if (rightSpeed > 0) {
            robot.rightMotor.forward();
        }
        else {
            robot.rightMotor.backward();
        }
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
        robot.leftMotor.rotate(-convertAngle(robot.leftRadius, angle), true);
        robot.rightMotor.rotate(convertAngle(robot.rightRadius, angle), false);

        stop();

        // if (!goodEnough(theta))
        //     turnTo(theta);

        // RESET TURNING FLAG
        turning = false;
    }

   /**
     * Turns the angle specified
     *
     * @param theta         orientation in radians
     */
    public void turn(double theta) {
        // CORRECT ANGLE TO REMAIN WITHIN -180 AND 180 DEGREES
        // TO MINIMIZE ANGLE TO SPIN
        if (theta < -3.14)
            theta += 6.28;
        else if (theta > 3.14)
            theta -= 6.28;

        // SET TURNING FLAG
        turning = true;

        // ROTATE SAID ANGLE AND WAIT UNTIL DONE
        robot.leftMotor.rotate(-convertAngle(robot.leftRadius, theta), true);
        robot.rightMotor.rotate(convertAngle(robot.rightRadius, theta), false);

        stop();

        if (!goodEnough(theta))
            turnTo(theta);

        // RESET TURNING FLAG
        turning = false;
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
     */
    public void avoid(int direction) {
        int counter = 0;

        switch (direction) {
            case 0:
                return;
            case CENTER:
                turn(-Math.PI / 2);
                avoid(LEFT);
                break;
            case LEFT:
                while (true) {
                    if (robot.leftSonic.getDistance() > 50) {
                        robot.leftMotor.setSpeed(HIGH);
                        robot.rightMotor.setSpeed(HIGH);

                        Delay.msDelay(5000);
                        break;
                    }

                    if (robot.leftSonic.getDistance() <= (BANDCENTRE + BANDWIDTH) && robot.leftSonic.getDistance() >= (BANDCENTRE + BANDWIDTH)) {
                        // IF SO KEEP GOING STRAIGHT
                        // EVERYTHING'S FINE
                        robot.leftMotor.setSpeed(HIGH);
                        robot.rightMotor.setSpeed(HIGH);
                    } else if (robot.leftSonic.getDistance() < (BANDCENTRE - BANDWIDTH)) {
                        // ELSE CHECK IF TOO CLOSE TO WALL
                        // AND IF SO TURN RIGHT BY A PRESET VALUE
                        // BY ROTATING THE LEFT WHEEL AT MOTORHIGH DEG/SEC
                        // AND THE RIGHT AT MOTORLOW DEG/SEC
                        // TO DISTANCE ITSELF FROM IT
                        robot.leftMotor.setSpeed(HIGH);
                        robot.rightMotor.setSpeed(LOW);
                    } else if (robot.leftSonic.getDistance() > (BANDCENTRE + BANDWIDTH)) {
                        // ELSE CHECK IF TOO FAR FROM WALL IN CONVEX CORNER
                        // AND IF SO TURN LEFT BY A PRESET VALUE
                        // BY ROTATING THE RIGHT WHEEL AT MOTORHIGH DEG/SEC
                        // AND BY ROTATING THE LEFT BACKWARDS AT MOTORHIGH DEG/SEC
                        // TO APPROACH IT BY PIVOTING ON ITSELF
                        robot.leftMotor.setSpeed(LOW);
                        robot.rightMotor.setSpeed(HIGH);
                    }
                }
                break;
            case RIGHT:
                while (true) {
                    if (robot.rightSonic.getDistance() > 50) {
                        robot.leftMotor.setSpeed(HIGH);
                        robot.rightMotor.setSpeed(HIGH);

                        Delay.msDelay(500);
                        break;
                    }

                    if (robot.rightSonic.getDistance() <= (BANDCENTRE + BANDWIDTH) && robot.rightSonic.getDistance() >= (BANDCENTRE + BANDWIDTH)) {
                        // IF SO KEEP GOING STRAIGHT
                        // EVERYTHING'S FINE
                        robot.leftMotor.setSpeed(HIGH);
                        robot.rightMotor.setSpeed(HIGH);
                    } else if (robot.rightSonic.getDistance() < (BANDCENTRE - BANDWIDTH)) {
                        // ELSE CHECK IF TOO CLOSE TO WALL
                        // AND IF SO TURN LEFT BY A PRESET VALUE
                        // BY ROTATING THE RIGHT WHEEL AT MOTORHIGH DEG/SEC
                        // AND THE LEFT AT MOTORLOW DEG/SEC
                        // TO DISTANCE ITSELF FROM IT
                        robot.leftMotor.setSpeed(LOW);
                        robot.rightMotor.setSpeed(HIGH);
                    } else if (robot.rightSonic.getDistance() > (BANDCENTRE + BANDWIDTH)) {
                        // ELSE CHECK IF TOO FAR FROM WALL IN CONVEX CORNER
                        // AND IF SO TURN RIGHT BY A PRESET VALUE
                        // BY ROTATING THE LEFT WHEEL AT MOTORHIGH DEG/SEC
                        // AND BY ROTATING THE LEFT BACKWARDS AT MOTORHIGH DEG/SEC
                        // TO APPROACH IT BY PIVOTING ON ITSELF
                        robot.leftMotor.setSpeed(HIGH);
                        robot.rightMotor.setSpeed(LOW);
                    }
                }
                break;
        }

        robot.leftMotor.setSpeed(HIGH);
        robot.rightMotor.setSpeed(HIGH);
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
     * @param distance      distance in centimeters
     *
     * @return angle (in degrees) each wheel should rotate
     */
    private static int convertAngle(double radius, double angle) {
        return convertDistance(radius, robot.width * angle / 2.0);
    }
}
