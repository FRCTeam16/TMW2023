// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DMS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.FlowControl;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.PartIndicator;

public class LEDSubsystem extends SubsystemBase implements Lifecycle {
    private boolean running = true;
    private Timer timer = new Timer();
    private SerialPort serial;

    private enum DMSPhase {
        Stopped, RunDriveMotors, RunSteerMotors, DisplayResults
    }
    private DMSPhase currentPhase = DMSPhase.Stopped;
    private DMSStats driveDmsStatus = new DMSStats();
    private DMSStats steerDmsStatus = new DMSStats();
    private DriveInfo<Integer> driveStatus = new DriveInfo<Integer>(0);
    private DriveInfo<Integer> steerStatus = new DriveInfo<Integer>(0);

    private static final double INITIAL_IGNORE_TIME = 1.0;
    private static final double MOTOR_TEST_TIME = 4.0;
    private static final double RESULTS_DISPLAY_TIME = 10.0;

    private int secondsToClimb = 30;

    private static final int BUFFER_SIZE = 15;

    /** Creates a new LEDSubsystem. */
    public LEDSubsystem() {
        SmartDashboard.setDefaultNumber("LEDClimbTime", 30);
        try {
            if (running) {
                serial = new SerialPort(57600, SerialPort.Port.kUSB1, 8, Parity.kNone, StopBits.kOne);
                serial.setWriteBufferSize(BUFFER_SIZE);
                serial.setWriteBufferMode(WriteBufferMode.kFlushWhenFull);
            }
        } catch (Exception e) {
            System.err.println("Unable to create DMS/LED subsystem, problem with serial port: " + e.getMessage());
            running = false;
        }
    }
    

    /**
     * Called out of band by a scheduled periodic in main robot
     */
    public void Report() {
        SmartDashboard.putBoolean("DMS/Running", running);
        SmartDashboard.putBoolean("DMS/HasSerial", (serial != null));
        if (running && serial != null) {
            try {
                SendData(new DriveInfo<Double>(0.0), new DriveInfo<Double>(0.0));
            } catch (Exception e) {
                // error sending data
                System.out.println("LED EXCEPTION: " + e.getMessage());
                running = false;
            } catch (Error e) {
                System.out.println("LED ERROR: " + e.getMessage());
                running = false;
            }
        }
    }

    public void SendData(DriveInfo<Double> driveMotor, DriveInfo<Double> steerMotor) {
        int robotState = 0;
        if (DriverStation.isDisabled()) {
            robotState = 1;
        } else if (DriverStation.isAutonomous()) {
            robotState = 2;
        } else if (DriverStation.isTeleop()) {
            robotState = 3;
        }

        int requestedPart = (int) SmartDashboard.getNumber(PartIndicator.KEY, 0);

        int allianceColor = 0;
        if (DriverStation.getAlliance() == Alliance.Red) {
            allianceColor = 1;
        } else if (DriverStation.getAlliance() == Alliance.Blue) {
            allianceColor = 2;
        }

        int partDetected = Subsystems.intake.isProxTripped() ? 1 : 0;

        int robotPitch = (int) Subsystems.swerveSubsystem.gyro.getPitch();


        byte[] buffer = new byte[BUFFER_SIZE];

        buffer[0] = (byte) 254;
        // DMS info
        buffer[1] = 0;
        buffer[2] = 0;
        buffer[3] = 0;
        buffer[4] = 0;
        buffer[5] = 0;
        buffer[6] = 0;
        buffer[7] = 0;
        buffer[8] = 0;

        buffer[9] = (byte) robotState; // comm status
        buffer[10] = (byte) allianceColor;
        buffer[11] = (byte) requestedPart;  // request part type
        buffer[12] = (byte) partDetected;
        buffer[13] = (byte) robotPitch;
        buffer[14] = (byte) 255;

        // System.out.println("[LED] Part Detected: " + (byte)partDetected);

        this.serial.write(buffer, buffer.length);
        this.serial.flush();
    }
    

    public void begin() {
        timer.reset();
    }
    
    public void startSubsystem() {
        running = true;
    }

    public void stopSubsystem() {
        running = false;
    }

    public void startDMS() {
        timer.reset();
        timer.start();
        driveDmsStatus = new DMSStats();
        steerDmsStatus = new DMSStats();

        driveStatus = new DriveInfo<Integer>(0);
        steerStatus = new DriveInfo<Integer>(0);

        currentPhase = DMSPhase.RunDriveMotors;
    }

    public boolean isStopped() {
        return currentPhase == DMSPhase.Stopped;
    }
    
    public void stopDMS() {
        System.out.println("Stopping DMS");
        currentPhase = DMSPhase.Stopped;
        
        driveStatus = new DriveInfo<Integer>(0);
        steerStatus = new DriveInfo<Integer>(0);
        timer.stop();
    }

    @Override
    public void periodic() {

        int climbTimeFromDashboard = (int)SmartDashboard.getNumber("LEDClimbTime", 30);
        if (climbTimeFromDashboard != secondsToClimb) {
            secondsToClimb = climbTimeFromDashboard;
        }

        if (running) {
            
            switch (currentPhase) {
                case Stopped:
                    break;
                case RunDriveMotors:
                System.out.println("Running DMS: " + currentPhase + " | " + timer.get());
                    runMotorTest();
                    break;
                case RunSteerMotors:
                System.out.println("Running DMS: " + currentPhase + " | " + timer.get());
                    runSteerTest();
                    break;
                case DisplayResults:
                System.out.println("Running DMS: " + currentPhase + " | " + timer.get());
                    displayResults();
                    break;

            }
            ;
        }
    }

    private void runMotorTest() {
        final double now = timer.get();
        if (now < MOTOR_TEST_TIME) {
            // FIXME
            // Subsystems.drivetrainSubsystem.DMSDrive(1.0);
            // Subsystems.drivetrainSubsystem.DMSSteer(0.0);

            if (now > INITIAL_IGNORE_TIME) {
                // FIXME
                // driveDmsStatus.addDriveCurrent(Subsystems.drivetrainSubsystem.getDriveOutputCurrent());
                // driveDmsStatus.addDriveVelocity(Subsystems.drivetrainSubsystem.getDriveVelocity());

                DMSStats.print("(DVel)", driveDmsStatus.velocity);
                DMSStats.print("(DAmp)", driveDmsStatus.current);

                double velAvg = DMSStats.average(driveDmsStatus.velocity);
                double ampAvg = DMSStats.average(driveDmsStatus.current);
                System.out.println("Vel Avg: " + velAvg + " | Amp Avg: " + ampAvg);
                
                driveStatus = driveDmsStatus.calculateStatus();
                DMSStats.print("[Drive Status]", driveStatus);
            }
        } else {
            SmartDashboard.putNumber("DMS/Result/FL/Drive/Status", driveStatus.FL);
            SmartDashboard.putNumber("DMS/Result/FL/Drive/Vel", driveDmsStatus.velocity.FL);
            SmartDashboard.putNumber("DMS/Result/FL/Drive/Amp", driveDmsStatus.current.FL);
            SmartDashboard.putNumber("DMS/Result/FR/Drive/Status", driveStatus.FR);
            SmartDashboard.putNumber("DMS/Result/FR/Drive/Vel", driveDmsStatus.velocity.FR);
            SmartDashboard.putNumber("DMS/Result/FR/Drive/Amp", driveDmsStatus.current.FR);
            SmartDashboard.putNumber("DMS/Result/RL/Drive/Status", driveStatus.RL);
            SmartDashboard.putNumber("DMS/Result/RL/Drive/Vel", driveDmsStatus.velocity.RL);
            SmartDashboard.putNumber("DMS/Result/RL/Drive/Amp", driveDmsStatus.current.RL);
            SmartDashboard.putNumber("DMS/Result/RR/Drive/Status", driveStatus.RR);
            SmartDashboard.putNumber("DMS/Result/RR/Drive/Vel", driveDmsStatus.velocity.RR);
            SmartDashboard.putNumber("DMS/Result/RR/Drive/Amp", driveDmsStatus.current.RR);

            currentPhase = DMSPhase.RunSteerMotors;
            timer.reset();
        }
    }

    private void runSteerTest() {
        final double now = timer.get();
        if (now < MOTOR_TEST_TIME) {
            // FIXME
            // Subsystems.drivetrainSubsystem.DMSDrive(0.0);
            // Subsystems.drivetrainSubsystem.DMSSteer(1.0);

            if (now > INITIAL_IGNORE_TIME) {
                // FIXME
                // steerDmsStatus.addDriveCurrent(Subsystems.drivetrainSubsystem.getSteerOutputCurrent());
                // steerDmsStatus.addDriveVelocity(Subsystems.drivetrainSubsystem.getSteerVelocity());

                DMSStats.print("(SVel)", steerDmsStatus.velocity);
                DMSStats.print("(SAmp)", steerDmsStatus.current);

                double velAvg = DMSStats.average(steerDmsStatus.velocity);
                double ampAvg = DMSStats.average(steerDmsStatus.current);
                System.out.println("Vel Avg: " + velAvg + " | Amp Avg: " + ampAvg);
                
                steerStatus = steerDmsStatus.calculateStatus();
                DMSStats.print("[Steer Status]", steerStatus);
            } 
        } else {
            // FIXME
            // Subsystems.drivetrainSubsystem.DMSDrive(0.0);
            // Subsystems.drivetrainSubsystem.DMSSteer(0.0);
            currentPhase = DMSPhase.DisplayResults;
            SmartDashboard.putNumber("DMS/Result/FL/Steer/Status", steerStatus.FL);
            SmartDashboard.putNumber("DMS/Result/FL/Steer/Vel", steerDmsStatus.velocity.FL);
            SmartDashboard.putNumber("DMS/Result/FL/Steer/Amp", steerDmsStatus.current.FL);
            SmartDashboard.putNumber("DMS/Result/FR/Steer/Status", steerStatus.FR);
            SmartDashboard.putNumber("DMS/Result/FR/Steer/Vel", steerDmsStatus.velocity.FR);
            SmartDashboard.putNumber("DMS/Result/FR/Steer/Amp", steerDmsStatus.current.FR);
            SmartDashboard.putNumber("DMS/Result/RL/Steer/Status", steerStatus.RL);
            SmartDashboard.putNumber("DMS/Result/RL/Steer/Vel", steerDmsStatus.velocity.RL);
            SmartDashboard.putNumber("DMS/Result/RL/Steer/Amp", steerDmsStatus.current.RL);
            SmartDashboard.putNumber("DMS/Result/RR/Steer/Status", steerStatus.RR);
            SmartDashboard.putNumber("DMS/Result/RR/Steer/Vel", steerDmsStatus.velocity.RR);
            SmartDashboard.putNumber("DMS/Result/RR/Steer/Amp", steerDmsStatus.current.RR);

            timer.reset();
        }
    }

    private void displayResults() {
        final double now = timer.get();
        if (now > RESULTS_DISPLAY_TIME) {
            currentPhase = DMSPhase.Stopped;
        }
    }

}
