/*
* @author Sean Lawlor
* @date November 3, 2011
* @class ECSE 211 - Design Principle and Methods
* 
* Modified by F.P. Ferrie
* February 28, 2014
* Changed parameters for W2014 competition
*/
package bluetooth;

import java.io.DataInputStream;
import java.io.IOException;

import lejos.nxt.LCD;

/*
 * Static parsers for parsing data off the communication channel
 * 
 * The order of data is defined in the Server's Transmission class
 */

public class ParseTransmission {
	
	public static Transmission parse (DataInputStream dis) {
		Transmission trans = null;
		try {
			
			while (dis.available() <= 0)
				Thread.sleep(10); // spin waiting for data
			
			trans = new Transmission();
			trans.role = PlayerRole.lookupRole(dis.readInt());
			ignore(dis);
			trans.startingCorner = StartCorner.lookupCorner(dis.readInt());
			ignore(dis);
			trans.greenZoneLL_X = dis.readInt();
			ignore(dis);
			trans.greenZoneLL_Y = dis.readInt();
			ignore(dis);
			trans.greenZoneUR_X = dis.readInt();
			ignore(dis);
			trans.greenZoneUR_Y = dis.readInt();
			ignore(dis);
			trans.redZoneLL_X = dis.readInt();
			ignore(dis);
			trans.redZoneLL_Y = dis.readInt();
			ignore(dis);
			trans.redZoneUR_X = dis.readInt();
			ignore(dis);
			trans.redZoneUR_Y = dis.readInt();
			ignore(dis);
			trans.greenDZone_X = dis.readInt();
			ignore(dis);
			trans.greenDZone_Y = dis.readInt();
			ignore(dis);
			trans.redDZone_X = dis.readInt();
			ignore(dis);
			trans.redDZone_Y = dis.readInt();
			ignore(dis);
			trans.greenFlag = dis.readInt();
			ignore(dis);
			trans.redFlag = dis.readInt();
			ignore(dis);			
			return trans;
		} catch (IOException e) {
			// failed to read transmitted data
			LCD.drawString("IO Ex", 0, 7);
			return trans;
		} catch (InterruptedException e) {
			return trans;
		}
		
	}
	
	public static void ignore(DataInputStream dis) throws IOException {
		dis.readChar();
	}
	
}
