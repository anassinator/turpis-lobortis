/**
 * Claw.java
 * Grabs, lifts and drops objects
 * @author  Anass Al-Wohoush
 * @version 0.5
 */

import lejos.nxt.*;
import lejos.util.*;

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
        claw.setSpeed(100);
        claw.rotate(4*180 - 45);
    }

    /**
     * Grabs object and lifts arm
     */
    public void grab() {
        claw.setSpeed(100);
        claw.rotate(-4*360);
    }
}
