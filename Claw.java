import lejos.nxt.*;
import lejos.util.*;
import java.util.*;

/**
 * Grabs, lifts and drops objects
 * @author  Anass Al-Wohoush
 * @version 0.6
 */
public class Claw {
    // OBJECTS
    public NXTRegulatedMotor claw;
    public boolean isDown = false;

    /**
     * Claw constructor
     *
     * @param claw       motor port controlling claw
     */
    public Claw(NXTRegulatedMotor claw) {
        this.claw = claw;
        claw.stop();
    }

    /**
     * Lowers arm and opens claw
     */
    public void drop() {
        isDown = true;
        claw.setSpeed(360);
        claw.rotate(8*180);
    }

    /**
     * Grabs object and lifts arm
     */
    public void grab() {
        claw.setSpeed(360);
        claw.rotate(-8*180);
        isDown = false;
    }
}
