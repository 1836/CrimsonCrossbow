package com.milkenknights.crimsoncrossbow;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;

public class Utils {
	/**
	 * Uses the Bang-Bang algorithm to return a power to send to the motor.
	 * Calculates RPM based on what source gives it
	 * @param targetRPM The desired RPM
	 * @param slowOutput What the function to return if the real RPM is greater than the target
	 * @param source The CounterBase that monitors the speed of the motor.
	 * @return slowOutput if current > target, or 1 if current < target
	 */
	public static double getBangBang(double targetRPM, double slowOutput, CounterBase source) {
		return periodToRPM(source.getPeriod()) < targetRPM ? 1 : slowOutput;
	}
	
	/**
	 * Converts a period value (time in seconds between two ticks) to RPM.
	 * @param periodInSeconds
	 * @return The RPM
	 */
	public static double periodToRPM(double periodInSeconds) {
		return 60/periodInSeconds;
	}
	
	/**
	 * @param frac The power
	 * @return The power value modified by battery voltage
	 */
	public static double voltageSpeed(double frac) {
		double voltage = DriverStation.getInstance().getBatteryVoltage();

		if (voltage != 0) {
			return (12.5*frac) / voltage;
		} else {
			return 0;
		}
	}
}
