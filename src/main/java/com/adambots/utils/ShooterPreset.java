package com.adambots.utils;

public class ShooterPreset implements Comparable<ShooterPreset>{
    private double armAngle;
    private double wristAngle;
    private double shootingSpeed;
    private double distance;

    public ShooterPreset(double armAngle, double wristAngle, double shootingSpeed, double distance) {
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.shootingSpeed = shootingSpeed;
        this.distance = distance;
    }

    public double getArmAngle() {
        return armAngle;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public double getShootingSpeed() {
        return shootingSpeed;
    }

    public double getDistance() {
        return distance;
    }

    public void setArmAngle(double armAngle) {
        this.armAngle = armAngle;
    }

    public void setWristAngle(double wristAngle) {
        this.wristAngle = wristAngle;
    }

    public void setShootingSpeed(double shootingSpeed) {
        this.shootingSpeed = shootingSpeed;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    @Override
    public int compareTo(ShooterPreset pPreset) {
        return Double.compare(this.getDistance(), pPreset.getDistance());
    }
}
