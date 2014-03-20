/**
 * Lobortis.java
 * Runs on slave NXT device. Create an LCP responder to handle LCP
 * requests over RS485
 * @author Andy Shaw, Anass Al-Wohoush
 * @version 0.6
 */

import lejos.nxt.*;
import lejos.nxt.comm.*;

public class Lobortis
{
    /**
     * Our local Responder class so that we can over-ride the standard
     * behaviour. We modify the disconnect action so that the thread will
     * exit.
     */
    static class Responder extends LCPResponder
    {
        Responder(NXTCommConnector con)
        {
            super(con);
        }

        protected void disconnect()
        {
            super.disconnect();
            super.shutdown();
        }
    }

    public static void main(String[] args) throws Exception
    {
        Sound.setVolume(Sound.VOL_MAX);
        Sound.systemSound(false, 3);
        LCD.clear();
        LCD.drawString("Running...", 0, 1);
        Responder resp = new Responder(RS485.getConnector());
        resp.start();
        resp.join();
        LCD.drawString("Closing...  ", 0, 1);
    }
}
