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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.ODOMETRY_FREQUENCY_HERTZ;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on the RIO. A Notifier thread is used to gather
 * samples with consistent timing. A single thread is used to update the signals and queues.*
 */
public class TalonFXOdometryThread6328 extends Thread {
    private final Lock odometryLock;
    private final Lock signalsLock = new ReentrantLock();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];

    private static TalonFXOdometryThread6328 INSTANCE = null;

    public static TalonFXOdometryThread6328 getInstance(Lock odometryLock) {
        if (INSTANCE == null) {
            INSTANCE = new TalonFXOdometryThread6328(odometryLock);
        }
        return INSTANCE;
    }

    private TalonFXOdometryThread6328(Lock odometryLock) {
        this.odometryLock = odometryLock;

        setName("PhoenixOdometryThread");
        setDaemon(true);
        start();
    }

    public Queue<Double> getTimestampQueue() {
        return timestamps;
    }

    public Queue<Double> registerSignal(StatusSignal<Double> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        signalsLock.lock();
        odometryLock.lock();

        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;

            signals = newSignals;
            queues.add(queue);
        } finally {
            signalsLock.unlock();
            odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        Timer.delay(5);

        //What could you do instead of while (true) ?
        while (true) {
            updateSignals();
            saveDataToQueues();
        }
    }

    private void updateSignals() {
        // Wait for updates from all signals
        signalsLock.lock();

        try {
            Thread.sleep((long) (1000.0 / ODOMETRY_FREQUENCY_HERTZ));

            if (signals.length > 0)
                BaseStatusSignal.refreshAll(signals);

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            DriverStation.reportError("Interrupted while waiting for signals", true);
        } finally {
            signalsLock.unlock();
        }

    }

    private void saveDataToQueues() {
        double fpgaTimestamp = HALUtil.getFPGATime() / 1.0e6; //Fuck you akit!

        // Save new data to queues
        odometryLock.lock();

        try {
            for (int i = 0; i < signals.length; i++) {
                queues.get(i).offer(signals[i].getValueAsDouble());
            }
            timestamps.offer(fpgaTimestamp);
        } finally {
            odometryLock.unlock();
        }
    }
}