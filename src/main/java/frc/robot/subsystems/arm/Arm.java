// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    private final ArmIO arm;
    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private static LoggedTunableNumber kP = new LoggedTunableNumber("CoralScoringArm/kP");
    private static LoggedTunableNumber kG = new LoggedTunableNumber("CoralScoringArm/kG");
    private static LoggedTunableNumber kV = new LoggedTunableNumber("CoralScoringArm/kV");
    private static LoggedTunableNumber kA = new LoggedTunableNumber("CoralScoringArm/kA", 0);
    private static LoggedTunableNumber kS = new LoggedTunableNumber("CoralScoringArm/kS", 0);
    private static LoggedTunableNumber kI = new LoggedTunableNumber("CoralScoringArm/kI", 0);

    private static double maxVelocityDegPerSec;
    private static double maxAccelerationDegPerSecSquared;

    private TrapezoidProfile armProfile;
    private TrapezoidProfile.Constraints armConstraints;

    private TrapezoidProfile.State armGoalStateDegrees = new TrapezoidProfile.State();
    private TrapezoidProfile.State armCurrentStateDegrees = new TrapezoidProfile.State();

    private double goalDegrees;

    private ArmFeedforward armFFModel;

    /**
     * Creates a new Arm.
     */
    public Arm(ArmIO arm) {
        this.arm = arm;
        kG.initDefault(0.08);
        kV.initDefault(1);
        kP.initDefault(0.7);
        kA.initDefault(0);
        kS.initDefault(0);
        kI.initDefault(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
