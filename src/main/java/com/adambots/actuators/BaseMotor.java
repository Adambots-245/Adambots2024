package com.adambots.actuators;

public interface BaseMotor {

    void set(double speed);

    void setInverted(boolean inverted);

    void setNeutralMode(boolean brake);

    void setPosition(double rotations);

    double getPosition();

    double getVelocity();

    double getAcceleration();

    double getCurrentDraw();

    boolean getForwardLimitSwitch();

    boolean getReverseLimitSwitch();

}