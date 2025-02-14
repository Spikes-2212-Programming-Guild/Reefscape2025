package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class PlaceCoralAndTakeAlgae extends SequentialCommandGroup {

    private static final double SPEED = -0.2;
    private static final double TIME = 0.5;

    public PlaceCoralAndTakeAlgae(Elevator elevator, AlgaeJoint algaeJoint, Gripper gripper,
                                  Drivetrain drivetrain, CoralJoint coralJoint, Storage storage,
                                  Elevator.ElevatorLevel level) {
        addCommands(new ParallelCommandGroup(
                        new MoveToHeight(elevator, level),
                        new RotateStorage(coralJoint,
                                CoralJoint.StoragePose.PLACEMENT).andThen(new ReleaseCoral(storage)),
                        new RotateAlgaeJointToBottom(algaeJoint)).andThen(new IntakeAlgae(gripper).withTimeout(TIME)),
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new Drive(drivetrain, () -> SPEED, () -> 0.0, () -> 0.0, false, false,
                                        false).withTimeout(TIME).andThen(new MoveToHeight(elevator,
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
