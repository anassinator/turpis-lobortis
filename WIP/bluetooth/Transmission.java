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

/*
 * Skeleton class to hold datatypes needed for final project
 * 
 * Simply all public variables so can be accessed with 
 * Transmission t = new Transmission();
 * int d1 = t.d1;
 * 
 * and so on...
 * 
 * Also the role is an enum, converted from the char transmitted. (It should never be
 * Role.NULL)
 */

public class Transmission {
	
	public PlayerRole role;
	public StartCorner startingCorner;
	public int greenZoneLL_X;
	public int greenZoneLL_Y;
	public int greenZoneUR_X;
	public int greenZoneUR_Y;
	public int redZoneLL_X;
	public int redZoneLL_Y;
	public int redZoneUR_X;
	public int redZoneUR_Y;
	public int greenDZone_X;
	public int greenDZone_Y;
	public int redDZone_X;
	public int redDZone_Y;
	public int greenFlag;
	public int redFlag;


	
}
