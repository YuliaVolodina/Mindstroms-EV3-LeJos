package Project;



import java.util.LinkedList;
import java.util.concurrent.TimeUnit;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author Team 3
 *
 * This class takes care of all navigation related action of the robor, including 
 * but not limited to traversing bidge/tunnel, turning, travelling to a location,
 * obstacle avoidance, etc...
 * 
 * @extends Thread
 */
public class Navigate extends Thread {

	static final EV3LargeRegulatedMotor leftMotor =  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	//static State state ;
	UltrasonicPoller poller;
	
	static String state;

	boolean tunnelFirst = true;

	private LinkedList<Integer> path;
	public static Odometer odometer;

	public Navigate(Odometer odometer){ //constructor
		this.odometer = odometer;
	}

	/**
	 * @author Team 3
	 * 
	 * run method that manages the state machine for the demo
	 * 
	 * @void
	 */
	@Override
	public void run() {

		leftMotor.setSpeed(120);
		rightMotor.setSpeed(120);
		
		odometer.setXYT(91.44, 30.48, 0);
		Search search = new Search(odometer);
		search.run();
		UltrasonicLocalizer usLoc = new UltrasonicLocalizer(odometer, UltrasonicLocalizer.LocType.FALLING_EDGE, ProjectMain.uspoller);
		usLoc.run();
		
		OdometryCorrection odoCorrect = new OdometryCorrection(odometer);
		odoCorrect.run();
		turnTo(-90);
		odoCorrect.detectLine();
		
		leftMotor.stop();
		rightMotor.stop();

		odometer.setXYT(7*30.48, 1*30.48, 270);
		if (tunnelFirst) {
			state = "tunnel";
		}
		else {
			state = "bridge";
		}

		if (state == "tunnel") {
			navToTunnel();
			odoCorrect.run();
			leftMotor.stop();
			rightMotor.stop();
			//odometer.setXYT((ProjectMain.TN_LL_X*30.48)+30.48/2, ProjectMain.TN_LL_Y-1, 0);
			tunnelTraversal2();
			state = "aftertunnel";
		}

		if (state == "bridge") {
			bridgeTraversal();
			odoCorrect.detectLine();
			turnTo(-90);
			odoCorrect.detectLine();
			//odometer.setXYT((ProjectMain.BR_UR_X-1)*30.48, (ProjectMain.BR_UR_Y+1)*30.48, 270);

			state = "afterbridge";
		}

		if (state == "aftertunnel") {
			state = "returnbridge";
		}

		if (state == "afterbridge") {
			returnBridge();
			state = "returnbridge";
		}

		if (state == "returnbridge") {
			returnBridge();
		}

	}

	/**
	 * Method that travels to the tunnel or the bridge and changes the state of the project at every step
	 * 
	 * @void
	 */
//	public void doNavigate() {
//		double coordX;
//		double coordY;
//		while (!path.isEmpty()) {
//			coordX = path.removeFirst();
//			coordY = path.removeFirst();
//			travelTo(coordX, coordY);
//			if (coordX == ProjectMain.BR_LL_X && coordY == ProjectMain.BR_LL_Y
//					&& ProjectMain.state != State.BRIDGE) {
//				ProjectMain.state = State.BRIDGE;
//				break;
//			} 
//			else if (coordX == ProjectMain.TN_LL_X  && coordY == ProjectMain.TN_LL_X 
//					&& ProjectMain.state != State.TUNNEL) {
//				ProjectMain.state = State.TUNNEL;
//				break;
//			} 
//			else if (coordX == ProjectMain.BR_UR_X && coordY == ProjectMain.BR_UR_Y
//					&& ProjectMain.state != State.BRIDGE) {
//				ProjectMain.state = State.BRIDGE;
//				break;
//			} 
//			else if (coordX == ProjectMain.TN_UR_X && coordY == ProjectMain.TN_UR_Y
//					&& ProjectMain.state != State.TUNNEL) {
//				ProjectMain.state = State.TUNNEL;
//				break;
//			} 
//			else if ((coordX == 1 && coordY == 11) || (coordX == 1 && coordY == 11)) {
//				ProjectMain.state = State.FINISHED;
//				break;
//			}
//		}
//	}

	/**
	 * travels to the coordinates 
	 * 
	 * @param pointX
	 * @param pointY
	 * @void
	 */
	public static void travelTo(double pointX, double pointY){
		double[] location = new double[3];
		location = odometer.getXYT();
		double acutalTheta = location[2];
		double actualX = location[0];
		double actualY = location[1];
		ProjectMain.odo_x = actualX;
		ProjectMain.odo_y = actualY;
		ProjectMain.odo_theta = acutalTheta;
		double finalX = pointX * ProjectMain.TILE_SIZE;
		double finalY = pointY * ProjectMain.TILE_SIZE;
		double yDiff = finalY-ProjectMain.odo_y;
		double xDiff = finalX-ProjectMain.odo_x;
		double angleCalc = Math.toDegrees(Math.atan2(xDiff,yDiff));
		double distFinal = Math.hypot(xDiff,yDiff);
		double angleFinal = angleCalc - ProjectMain.odo_theta;
		if(angleFinal < -180){ 
			turnTo(angleFinal + 360);
		}
		else if(angleFinal > 180){
			turnTo(angleFinal - 360);
		}
		else{
			turnTo(angleFinal);
		}
		goTo(distFinal);
	}

	/**
	 * travels to the coordinates, uses turnTo2 (different track value)
	 * 
	 * @param pointX
	 * @param pointY
	 * @void
	 */
	public static void travelTo2(double pointX, double pointY){
		double[] location = new double[3];
		location = odometer.getXYT();
		double acutalTheta = location[2];
		double actualX = location[0];
		double actualY = location[1];
		ProjectMain.odo_x = actualX;
		ProjectMain.odo_y = actualY;
		ProjectMain.odo_theta = acutalTheta;
		double finalX = pointX * ProjectMain.TILE_SIZE;
		double finalY = pointY * ProjectMain.TILE_SIZE;
		double yDiff = finalY-ProjectMain.odo_y;
		double xDiff = finalX-ProjectMain.odo_x;
		double angleCalc = Math.toDegrees(Math.atan2(xDiff,yDiff));
		double distFinal = Math.hypot(xDiff,yDiff);
		double angleFinal = angleCalc - ProjectMain.odo_theta;
		if(angleFinal < -180){ 
			turnTo2(angleFinal + 360);
		}
		else if(angleFinal > 180){
			turnTo2(angleFinal - 360);
		}
		else{
			turnTo2(angleFinal);
		}
		goTo(distFinal);
	}


	/**
	 * Turn to, adjust heading of the robot to the desired destination.
	 *
	 * @param theta
	 *            the theta
	 */
	public static void turnTo(double theta){
		//		Lab5.leftMotor.setSpeed(Lab5.ROTATE_SPEED);
		//		Lab5.rightMotor.setSpeed(Lab5.ROTATE_SPEED);
		leftMotor.rotate(convertAngle(ProjectMain.WHEEL_RAD, ProjectMain.TRACK, theta), true);
		rightMotor.rotate(-convertAngle(ProjectMain.WHEEL_RAD, ProjectMain.TRACK, theta), false);
	}

	/**
	 * turnTo method with different track used for travelTo
	 * 
	 * @param theta
	 * 			theta to turn to
	 */
	public static void turnTo2(double theta){
		//		Lab5.leftMotor.setSpeed(Lab5.ROTATE_SPEED);
		//		Lab5.rightMotor.setSpeed(Lab5.ROTATE_SPEED);
		leftMotor.rotate(convertAngle(ProjectMain.WHEEL_RAD, 14.5, theta), true);
		rightMotor.rotate(-convertAngle(ProjectMain.WHEEL_RAD, 14.5, theta), false);
	}


	/**
	 * Turn to angle theta, but run the code that comes after (end with true in rotate statement for both motors)
	 *
	 * @param theta
	 *            the theta we want to turn to
	 *       
	 * @void
	 */
	static void turn(double theta) {
		leftMotor.rotate(convertAngle(ProjectMain.WHEEL_RAD, ProjectMain.TRACK, theta), true);
		rightMotor.rotate(-convertAngle(ProjectMain.WHEEL_RAD, ProjectMain.TRACK, theta), true);
	}

	/**
	 * set a boolean to know when it is navigating
	 * 
	 * @return boolean
	 */
	public static boolean isNavigating() {
		return leftMotor.isMoving() && rightMotor.isMoving();
	}

	/**
	 * Convert distance
	 *
	 * @param radius
	 *            the radius, wheel radius of robot
	 * @param distance
	 *            the distance to travel
	 * @return rotations, the number of rotations each wheel has to turn to adjust
	 *         heading to travel the distance
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Convert angle.
	 *
	 * @param radius
	 *            the radius, wheel radius of robot
	 * @param width
	 *            the width, distance between the wheels of the robot
	 * @param angle
	 *            the angle, the angle we want the robot to turn to
	 * @return rotations, the number of rotations each wheel has to turn to adjust
	 *         heading to theta
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * @author Team 3
	 * 
	 * Set the poller value in the UltrasonicPoller class
	 * 
	 * @param poller
	 * @void
	 */
	public void setPoller(UltrasonicPoller poller) {
		this.poller = poller;
	}

	/**
	 * @author Team 3
	 * 
	 * set the list of coordinates that the robot has to go to
	 * 
	 * @param coordinates the robot has to go to
	 * @void
	 */
	public void setPath(LinkedList <Integer> coordinates) {
		this.path = coordinates;
	}

	/**
	 * @author Kartik Misra
	 * 
	 * travel to bridge, only for bridge first, not when returning 
	 * 
	 * @void
	 */
	public void goToBridge() {
		travelTo(ProjectMain.BR_UR_X-0.5, ProjectMain.BR_UR_Y+1);
	}

	/**
	 * Reset the motors to th default forward speed of 150
	 * 
	 * @void
	 */
	public static void resetSpeed() {
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
	}

	/**
	 * Method that travels the bridge taking in consideration the speed bumps
	 * and odometer errors
	 * 
	 * @void
	 */
	public static void bridgeTraversal() {
		//		Lab5.leftMotor.setSpeed(50);
		//		Lab5.rightMotor.setSpeed(50);
		travelTo(ProjectMain.BR_LL_X+0.5, ProjectMain.BR_LL_Y-1);
		travelTo(ProjectMain.BR_UR_X-0.5, ProjectMain.BR_UR_Y+1);
		//resetSpeed();
		state = "returnbridge";
	}

	/**
	 * Method that travels the bridge on the way back only
	 * 
	 * @throws InterruptedException
	 * @void
	 */
	public static void returnBridge() {
		//		Lab5.leftMotor.stop();
		//		Lab5.rightMotor.stop();
		//TimeUnit.SECONDS.sleep(1);
		//		Lab5.leftMotor.setSpeed(50);
		//		Lab5.rightMotor.setSpeed(50);
		travelTo(ProjectMain.BR_UR_X-0.5, ProjectMain.BR_UR_Y+1);
		travelTo(ProjectMain.BR_LL_X+0.5, ProjectMain.BR_LL_Y-1);
		//resetSpeed();
		travelTo(ProjectMain.ST_X, ProjectMain.ST_Y);
		//TODO for final project
	}

	/**
	 * Method that travels to the tile before to tunnel
	 * @throws InterruptedException 
	 * 
	 */
	public static void navToTunnel() {
		//		Lab5.leftMotor.stop();
		//		Lab5.rightMotor.stop();
		//TimeUnit.SECONDS.sleep(1);
		travelTo2(ProjectMain.TN_LL_X+0.5, ProjectMain.TN_LL_Y-1);
		turnTo(360-odometer.getTheta());
		//		Lab5.leftMotor.stop();
		//		Lab5.rightMotor.stop();
		Sound.beep();
	}

	/**
	 * Method that travels the tunnel
	 * @throws InterruptedException 
	 * 
	 */
	public static void tunnelTraversal2() {
		travelTo2(ProjectMain.TN_UR_X-0.5, ProjectMain.TN_UR_Y+1);
		Sound.beep();
		//		Lab5.leftMotor.stop();
		//		Lab5.rightMotor.stop();
		state = "returnbridge";
	}

	/**
	 * Turn to method used in light localization
	 * @param theta
	 */

	public static void turnToLoc(double theta){
		//		Lab5.leftMotor.setSpeed(50);
		//		Lab5.rightMotor.setSpeed(50);
		leftMotor.rotate(convertAngle(ProjectMain.WHEEL_RAD, ProjectMain.TRACK, theta), true);
		rightMotor.rotate(-convertAngle(ProjectMain.WHEEL_RAD, ProjectMain.TRACK, theta), false);
	}

	/**
	 * Travels a distance with no angle changes
	 * @param distance
	 */
	public static void goTo(double distance) {
		leftMotor.forward();
		leftMotor.forward();
		//		Lab5.leftMotor.setSpeed(Lab5.FORWARD_SPEED);
		//		Lab5.rightMotor.setSpeed(Lab5.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(ProjectMain.WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(ProjectMain.WHEEL_RAD, distance), false);
	}

	/**
	 * TODO EXPORTS THE SEARCHING ALGORITHM TO NAVIGATION CLASS
	 */

}