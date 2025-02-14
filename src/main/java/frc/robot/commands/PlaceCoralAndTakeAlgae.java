package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class PlaceCoralAndTakeAlgae extends SequentialCommandGroup {

    private static final double DRIVE_BACKWARDS_SPEED = -0.2;
    private static final double DRIVE_TIMEOUT = 0.5;
    private static final double GRIPPER_TIMEOUT = 0.5;

    public PlaceCoralAndTakeAlgae(Elevator elevator, AlgaeJoint algaeJoint, Gripper gripper,
                                  Drivetrain drivetrain, CoralJoint coralJoint, Storage storage,
                                  Elevator.ElevatorLevel level) {
        addCommands(new ParallelCommandGroup(
                        new MoveToHeight(elevator, level),
                        new RotateStorage(coralJoint,
                                CoralJoint.StoragePose.PLACEMENT),
                        new RotateAlgaeJointToBottom(algaeJoint)),
                new ParallelCommandGroup(
                        new ReleaseCoral(storage), new IntakeAlgae(gripper).withTimeout(GRIPPER_TIMEOUT)
                ),
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new Drive(drivetrain, () -> DRIVE_BACKWARDS_SPEED, () -> 0.0, () -> 0.0, false, false,
                                        false).withTimeout(DRIVE_TIMEOUT).andThen(new MoveToHeight(elevator,
                                        Elevator.ElevatorLevel.PROCESSOR)),
                                new RotateStorage(coralJoint, CoralJoint.StoragePose.RESTING)
                        ),
                        new ParallelCommandGroup(
                                new RotateAlgaeJointToTop(algaeJoint),
                                new RotateStorage(coralJoint, CoralJoint.StoragePose.INTAKE),
                                new MoveToHeight(elevator, Elevator.ElevatorLevel.FEEDER)
                        ), gripper::hasAlgae));

    }
}
