package com.milkenknights.crimsoncrossbow;

// TODO: add comments to this file
public class SolenoidXORPair extends SolenoidPair {
	public SolenoidXORPair(int a, int b) {
		this(a, b, true);
	}

	public SolenoidXORPair(int a, int b, boolean initial) {
		super(a, b, initial);
	}

	public void set(boolean on) {
		super.set(on);
		sa.set(get());
		sb.set(!get());
	}

}
