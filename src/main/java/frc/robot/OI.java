package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class OI /*GEVALD*/ {

    public enum Side {

        LEFT, RIGHT;
    }

    private static final RootNamespace namespace = new RootNamespace("oi");
    private static final Supplier<Double> POWER = namespace.addConstantDouble("power", 2);
    private static final Supplier<Double> POWER_DEADBAND = namespace.addConstantDouble("power", 0.3);
    private static final Supplier<Double> ALGAE_REMOVAL_SPEED = namespace.addConstantDouble("algae removal speed", 0.3);

    private final PlaystationControllerWrapper driverJoystick = new PlaystationControllerWrapper(0);
    //    private final PlaystationControllerWrapper driverJoystick = new PlaystationControllerWrapper(1);
    private final PlaystationControllerWrapper navigatorJoystick = new PlaystationControllerWrapper(0);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);

    private final Elevator elevator = Elevator.getInstance();
    private final Storage storage = Storage.getInstance();
    //    private final CoralJoint coralJoint = CoralJoint.getInstance();
    private final CoralJoint coralJoint = CoralJoint.getInstance();
    private final AlgaeJoint algaeJoint = AlgaeJoint.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    private boolean inAlgaeMode;

    private double currentSetpoint;

    public OI() {
        navigatorJoystick.getL1Button().onTrue(
                new MoveToHeight(elevator, Elevator.ElevatorLevel.L1).andThen(
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.L1))
        );
        navigatorJoystick.getR1Button().onTrue(
                new MoveToHeight(elevator, Elevator.ElevatorLevel.L2).andThen(
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.L2))
        );
        navigatorJoystick.getR2Button().onTrue(
                new MoveToHeight(elevator, Elevator.ElevatorLevel.L3).andThen(
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.L3)
                ));
        navigatorJoystick.getL2Button().onTrue(new Reset(elevator, coralJoint));

        navigatorJoystick.getSquareButton().onTrue(
                new MoveToHeight(elevator, Elevator.ElevatorLevel.FEEDER).andThen(
                        new RotateStorage(coralJoint, CoralJoint.StoragePose.INTAKE))
        );
        navigatorJoystick.getTriangleButton().onTrue(new IntakeCoral(storage));
        navigatorJoystick.getCircleButton().onTrue(new ReleaseCoral(storage));
        navigatorJoystick.getCrossButton().whileTrue(new MoveGenericSubsystem(algaeJoint, ALGAE_REMOVAL_SPEED));

        navigatorJoystick.getShareButton().whileTrue(new RunCommand(storage::intake) {
            @Override
            public void end(boolean interrupted) {
                storage.stop();
            }
        });
        navigatorJoystick.getOptionsButton().onTrue(new MoveToHeight(elevator, Elevator.ElevatorLevel.UPPER_ALGAE));

        navigatorJoystick.getUpButton().whileTrue(new MoveGenericSubsystem(elevator, Elevator.ELEVATOR_FORWARD_SPEED));
        navigatorJoystick.getDownButton().whileTrue(new MoveGenericSubsystem(elevator, Elevator.ELEVATOR_BACKWARD_SPEED));
        navigatorJoystick.getLeftButton().whileTrue(new MoveGenericSubsystem(coralJoint, CoralJoint.CORAL_JOINT_BACKWARD_SPEED));
        navigatorJoystick.getRightButton().whileTrue(new MoveGenericSubsystem(coralJoint, CoralJoint.CORAL_JOINT_FORWARD_SPEED));

        navigatorJoystick.getRightStickButton().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
        navigatorJoystick.getLeftStickButton().whileTrue(new MoveGenericSubsystem(storage, -0.2));
//        driverJoystick.getR1Button().onTrue(new InstantCommand(drivetrain::resetGyro));
        new JoystickButton(rightJoystick, 1).onTrue(new InstantCommand(drivetrain::resetGyro));
        ConditionalCommand centerOnLeft = new CenterOnReef(drivetrain, Side.LEFT).onlyIf(
                () -> Math.abs(drivetrain.getRobotRelativePose().getX()) < 1 && Math.abs(drivetrain.getRobotRelativePose().getY()) < 1
        );
        ConditionalCommand centerOnRight = new CenterOnReef(drivetrain, Side.RIGHT).onlyIf(
                () -> Math.abs(drivetrain.getRobotRelativePose().getX()) < 1 && Math.abs(drivetrain.getRobotRelativePose().getY()) < 1
        );
        new JoystickButton(leftJoystick, 3).onTrue(centerOnLeft);
        new JoystickButton(leftJoystick, 4).onTrue(centerOnRight);
        new JoystickButton(rightJoystick, 2).onTrue(new InstantCommand(() -> {
            CommandScheduler.getInstance().cancel(centerOnLeft);
            CommandScheduler.getInstance().cancel(centerOnRight);
        }));
    }

    public double getLeftX() {
//        double val = driverJoystick.getLeftX();
        double val = leftJoystick.getX();
        return Math.abs(val) <= POWER_DEADBAND.get() ? Math.signum(val) * Math.pow(Math.abs(val), POWER.get()) : val;
    }

    public double getLeftY() {
//        double val = driverJoystick.getLeftY();
        double val = leftJoystick.getY();
        return Math.abs(val) <= POWER_DEADBAND.get() ? Math.signum(val) * Math.pow(Math.abs(val), POWER.get()) : val;
    }

    public double getRightX() {
//        double val = driverJoystick.getRightX();
        double val = rightJoystick.getX();
        return Math.abs(val) <= POWER_DEADBAND.get() ? Math.signum(val) * Math.pow(Math.abs(val), POWER.get()) : val;
    }

    public double getRightY() {
//        double val = driverJoystick.getRightY();
        double val = rightJoystick.getY();
        return Math.abs(val) <= POWER_DEADBAND.get() ? Math.signum(val) * Math.pow(Math.abs(val), POWER.get()) : val;
    }
}
