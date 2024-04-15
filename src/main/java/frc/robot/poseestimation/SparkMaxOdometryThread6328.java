// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.poseestimation;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ODOMETRY_FREQUENCY_HERTZ;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread6328 extends Thread {
    private final Lock odometryLock;
    private final List<DoubleSupplier> signals = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);

    public SparkMaxOdometryThread6328(Lock odometryLock) {
        this.odometryLock = odometryLock;

        try (Notifier notifier = new Notifier(this::periodic)) {
            notifier.setName("SparkMaxOdometryThread");
            Timer.delay(1);
            notifier.startPeriodic(1.0 / ODOMETRY_FREQUENCY_HERTZ);
        }
    }

    public Queue<Double> getTimestampQueue() {
        return timestamps;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        odometryLock.lock();

        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            odometryLock.unlock();
        }

        return queue;
    }

    private void periodic() {
        odometryLock.lock();
        timestamps.offer(HALUtil.getFPGATime() / 1.0e6);

        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
        } finally {
            odometryLock.unlock();
        }
    }
}