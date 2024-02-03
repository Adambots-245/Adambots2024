// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.Tests;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;
import java.util.List;

// import com.spartronics4915.lib.subsystems.estimator.RobotStateMap;
// import com.spartronics4915.lib.util.Units;

import org.opencv.core.Mat;

import com.adambots.sensors.RPLidarA1;
import com.adambots.utils.Units;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

public class TestObjectFinder {

    private List<Translation2d> mPointcloud = new ArrayList<>();
    private long mLastResetTime = 0;

    private int mEdgeDetectorValue = 4;
    private int mNumVotesNeeded = 5;

    private final TargetTracker mTargetTracker = new TargetTracker();

    public static void main(String[] args) {
        new TestObjectFinder().interactivePointcloudTest();
    }

    public void interactivePointcloudTest() {
        
        final ObjectFinder finder = new ObjectFinder(0.01);
        final double circleRadiusMeters = Units.inchesToMeters(3.5);

        var pointCloudCanvas = new Canvas() {
            @Override
            public void paint(Graphics g) {
                super.paint(g);

                try {
                    synchronized (mPointcloud) {
                        if (mPointcloud.size() <= 0) {
                            return;
                        }

                        mPointcloud.forEach((p) -> drawPoint(p, 0, g));

                        // int circleDiameterCentimeters = (int) Math.round(circleRadiusMeters * 100.0 * 2.0);
                        // var centers = finder.findSquares(mPointcloud, new Translation2d(), 0.01, mNumVotesNeeded, 5, 0.5);
                        var centers = finder.findCircles(mPointcloud, circleRadiusMeters, mNumVotesNeeded, 3);
                        /**
                        for (Translation2d center : centers) {
                            g.setColor(Color.GREEN);
                            g.drawOval(toScreenCoordinates(center.getX() - circleRadiusMeters, true), toScreenCoordinates(center.getY() - circleRadiusMeters, false), 50, 50);
                            // break;
                        }
                        */ 
                        var center = mTargetTracker.update(centers);
                        if (center == null) {
                            System.out.println("No target found");
                            return;
                        }
                        for (Translation2d center2 : centers) {
                            System.out.println(center2);
                            g.setColor(Color.BLUE);
                            g.drawOval(toScreenCoordinates(center2.getX() - circleRadiusMeters, true), toScreenCoordinates(center2.getY() - circleRadiusMeters, false), 50, 50);
                            // break;
                        }

                        g.drawString("Edge: " + mEdgeDetectorValue + ", Votes: " + mNumVotesNeeded, 10, 10);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            // Screen coordinates correspond to centimeters
            private int toScreenCoordinates(double coordMeters, boolean isX) {
                Dimension size = this.getSize();

                coordMeters *= 100;
                coordMeters += (isX ? size.getWidth() : size.getHeight()) / 2;

                return (int) Math.round(coordMeters);
            }

            private void drawPoint(Translation2d point, int quality, Graphics g) {
                g.setColor(new Color(255 - quality, 0, 0));
                g.drawOval(toScreenCoordinates(point.getX(), true), toScreenCoordinates(point.getY(), false), 2, 2);
            }
        };
        pointCloudCanvas.setSize(6000, 6000);
        pointCloudCanvas.setBackground(Color.WHITE);

        var frame = new Frame();
        frame.add(pointCloudCanvas);
        frame.setSize(6000, 6000);
        frame.setVisible(true);

        frame.addKeyListener(new KeyListener() {

            @Override
            public void keyPressed(KeyEvent k) {
            }

            @Override
            public void keyReleased(KeyEvent k) {
                System.out.println(k.getKeyChar());
                if (k.getKeyChar() == '-' || k.getKeyChar() == '+') {
                    double toAdd = (k.getKeyChar() == '+' ? 1 : -1);

                    if (k.isAltDown()) {
                        mEdgeDetectorValue += toAdd;
                    } else if (k.isControlDown()) {
                        mNumVotesNeeded += toAdd;
                    }
                } else if (k.getKeyChar() == 'q') {
                    System.exit(0);
                }
            }

            @Override
            public void keyTyped(KeyEvent k) {
            }

        });

        RPLidarA1 lidar = new RPLidarA1();
        lidar.setCallback((List<Translation2d> pointcloud) -> {
            synchronized (mPointcloud) {
                if (System.currentTimeMillis() - mLastResetTime > 0) {
                    mPointcloud = pointcloud;
                    mLastResetTime = System.currentTimeMillis();
                } else {
                    mPointcloud.addAll(pointcloud);
                }
            }
        // }, new RobotStateMap(), new Transform2d(new Translation2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-14)), Rotation2d.fromDegrees(180)));
    });
        lidar.start();

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            lidar.stop();
            System.out.println("Graceful shutdown complete");
        }));

        while (true) {
            try {
                Thread.sleep(1000);
                synchronized (mPointcloud) {
                    pointCloudCanvas.repaint();                    
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}
