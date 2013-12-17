package com.milkenknights.crimsoncrossbow;

import edu.wpi.first.wpilibj.Timer;

public class Pulse {
	private double pulseTime, delay;
	private double lastShootTime;
	private boolean isShooting;

	public Pulse(double pulseTime, double delay) {
		this.pulseTime = pulseTime;
		this.delay = delay;
	}

	public double getPulse(double desiredSpeed) {
		if (desiredSpeed == 0) {
			isShooting = false;
			return 0;
		} else {
			if (isShooting) {
				if (Timer.getFPGATimestamp() - lastShootTime >= pulseTime + delay) {
					lastShootTime = Timer.getFPGATimestamp();
				} else if (Timer.getFPGATimestamp() - lastShootTime > pulseTime) {
					return 0;
				} else {
					return desiredSpeed;
				}
			} else {
				isShooting = true;
				lastShootTime = Timer.getFPGATimestamp();
			}
		}
		// TODO: delete this return and make sure
		// returns work everywhere
		return desiredSpeed;
	}
}
