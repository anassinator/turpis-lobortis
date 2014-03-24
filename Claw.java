import lejos.nxt.*;
import lejos.util.*;

/**
 * Grabs, lifts and drops objects
 * @author  Anass Al-Wohoush
 * @version 0.5
 */
public class Claw {

    // SLAVE NXT
    public NXTRegulatedMotor claw;

    /**
     * Claw constructor
     *
     * @param robot             robot object containing all color sensor ports
     */
    public Claw(Robot robot) {
        this.claw = robot.claw;
    }

    /**
     * Lowers arm and opens claw
     */
    public void drop() {
        robot.clawIsDown = true;
        claw.setSpeed(360);
        claw.rotate(8*180);
    }

    /**
     * Grabs object and lifts arm
     */
    public void grab() {
        claw.setSpeed(360);
        claw.rotate(-8*180);
        robot.clawIsDown = false;
    }
}
