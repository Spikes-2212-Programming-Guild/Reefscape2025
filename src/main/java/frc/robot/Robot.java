// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.autonomous.DriveAndPlaceL2;
import frc.robot.commands.autonomous.JustDrive;
import frc.robot.subsystems.AlgaeJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.district2.District2CoralJoint;

public class Robot extends TimedRobot {

//    @TODO remove all necessary actions that are useless to drive tests
    private PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
//    private Joystick left = new Joystick(1);
//    private Joystick right = new Joystick(2);


    RootNamespace namespace = new RootNamespace("robot");
    private Drivetrain drivetrain;
//    private Elevator elevator;
    private Storage storage;
//    private Gripper gripper;
    private District2CoralJoint coralJoint;
//    private AlgaeJoint algaeJoint;

    @Override
    public void robotInit() {
//        CameraServer.startAutomaticCapture(0);
//        CameraServer.startAutomaticCapture(1);
//        CvSink cvSink = CameraServer.getVideo();
//        CvSource outputStream = CameraServer.putVideo("camera", 720, 1280);
        drivetrain = Drivetrain.getInstance();
//        new JoystickButton(right, 1).onTrue(new InstantCommand(drivetrain::resetGyro));
        getInstances();
//        namespace.putCommand("L3", new PlaceOnL3(drivetrain, coralJoint, storage));
//        registerNamedCommands();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        coralJoint.calibrateEncoderPosition();
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
//        drivetrain.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
//        JustDrive auto = new JustDrive();
//        DriveAndPlaceL2 auto = new DriveAndPlaceL2();
//        auto.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        OI oi = new OI();
        drivetrain.resetRelativeEncoders();
//        drivetrain.setNeutralMode(NeutralModeValue.Coast);
        drivetrain.setDefaultCommand(new Drive(drivetrain, () -> -oi.getLeftY() * 4, () -> -oi.getLeftX() * 4, () -> oi.getRightX() * 6,
                true, false, false));
//        algaeJoint.setDefaultCommand(new MoveGenericSubsystem(algaeJoint, AlgaeJoint.STABILIZATION_SPEED).onlyIf(gripper::hasAlgae));
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    private void registerNamedCommands() {
//        NamedCommands.registerCommand("OuttakeCoralAngle",
//                new RotateStorage(coralJoint, CoralJoint.StoragePose.PLACEMENT));
//        NamedCommands.registerCommand("ReleaseCoral", new ReleaseCoral(storage));
//        NamedCommands.registerCommand("ElevateToFeeder", new MoveToHeight(elevator, Elevator.ElevatorLevel.FEEDER));
//        NamedCommands.registerCommand("IntakeCoralAngle", new RotateStorage(coralJoint, CoralJoint.StoragePose.INTAKE));
//        NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(storage));
//        NamedCommands.registerCommand("IntakeAlgaeAngle", new RotateAlgaeJointToBottom(algaeJoint));
//        NamedCommands.registerCommand("ElevateToL3", new MoveToHeight(elevator, Elevator.ElevatorLevel.L3));
//        NamedCommands.registerCommand("TakeAlgae", new IntakeAlgae(gripper));
//        NamedCommands.registerCommand("PlaceAlgae", new ReleaseAlgae(gripper));
//        NamedCommands.registerCommand("ElevateToL4", new MoveToHeight(elevator, Elevator.ElevatorLevel.L4));
    }

    private void getInstances() {
//        elevator = Elevator.getInstance();
        storage = Storage.getInstance();
//        gripper = Gripper.getInstance();
        coralJoint = District2CoralJoint.getInstance();
//        algaeJoint = AlgaeJoint.getInstance();
    }
}
