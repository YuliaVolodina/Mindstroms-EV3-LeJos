package Project;

//import lejos.robotics.SampleProvider;
//
///**
// * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
// * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
// * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
// * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
// * or about 14 Hz.
// */
//public class UltrasonicPoller extends Thread {
//  private SampleProvider us;
//  private UltrasonicController cont;
//  private float[] usData;
//private Navigate gps;
//
//  public UltrasonicPoller(SampleProvider us, float[] usData,  UltrasonicController cont) {
//    this.us = us;
//    this.gps = gps;
//    gps.setPoller(this);
//    this.usData = usData;
//    
//  }
//
//  /*
//   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
//   * [0,255] (non-Javadoc)
//   * 
//   * @see java.lang.Thread#run()
//   */
//  public void run() {
//    int distance;
//    while (true) {
//      us.fetchSample(usData, 0); // acquire data
//      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
//      cont.processUSData(distance); // now take action depending on value
//      try {
//        Thread.sleep(50);
//      } catch (Exception e) {
//      } // Poor man's timed sampling
//    }
//  }
//
//}

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller
 * thread. The while loop at the bottom executes in a loop. Assuming that the
 * us.fetchSample, and cont.processUSData methods operate in about 20mS, and
 * that the thread sleeps for 50 mS at the end of each loop, then one cycle
 * through the loop is approximately 70 mS. This corresponds to a sampling rate
 * of 1/70mS or about 14 Hz.
 * 
 * @version 1.0
 */
public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private UltrasonicController cont;
	private float[] usData;
	private Navigate gps;
	private int reading;
	private boolean on;

	public UltrasonicPoller(SampleProvider us, float[] usData, Navigate gps) {
		this.us = us;
		this.usData = usData;
		this.gps = gps;
		gps.setPoller(this);
	}

	/**
	 * Sensors now return floats using a uniform protocol. Need to convert US result
	 * to an integer [0,255]
	 * 
	 * @svoid
	 */
	public void run() {
		while (true) {
			if (on) {
				us.fetchSample(usData, 0); // acquire data
				reading = (int) (usData[0] * 100.0); // extract from buffer, cast to int
				if (reading > 255)
					reading = 255;
				try {
					Thread.sleep(50);
				} catch (Exception e) {
				} // Poor man's timed sampling
			} else {
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
				}
			}

		}
	}

	public void on() {
		on = true;
	}

	public void off() {
		on = false;
	}

	public int getReading() {
		return reading;
	}

}
