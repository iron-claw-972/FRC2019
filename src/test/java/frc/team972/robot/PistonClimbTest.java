package frc.team972.robot;

import org.junit.Test;

import frc.team972.robot.subsystems.PistonClimbSubsystem;
import junit.framework.Assert;

public class PistonClimbTest 
{

    @Test
    public void testPistonClimb()
    {
        System.out.println("PistonClimb test starts");
        PistonClimbSubsystem.getInstance().setPistonClimbNotTesting(false);
        System.out.println("PistonClimb test mode active");
        PistonClimbSubsystem.getInstance().setTime(5); //Tests all piston extension stages to see if they return completion or progression successfully
        System.out.println(PistonClimbSubsystem.getInstance().climbStage1(4));
        System.out.println(PistonClimbSubsystem.getInstance().climbStage1(4) == PistonClimbSubsystem.stageState.COMPLETE);
        
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage1(4) == PistonClimbSubsystem.stageState.COMPLETE);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage1(6) == PistonClimbSubsystem.stageState.IN_PROG);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage3(4) == PistonClimbSubsystem.stageState.COMPLETE);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage3(6) == PistonClimbSubsystem.stageState.IN_PROG);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage4(4) == PistonClimbSubsystem.stageState.COMPLETE);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage4(6) == PistonClimbSubsystem.stageState.IN_PROG);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage6(4) == PistonClimbSubsystem.stageState.COMPLETE);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage6(6) == PistonClimbSubsystem.stageState.IN_PROG);
		
		
        PistonClimbSubsystem.getInstance().setDetectionTime(3);
        PistonClimbSubsystem.getInstance().setRange(0); //Tests what happens if robot is in theory successfully mounting lip onto platform
        PistonClimbSubsystem.getInstance().setTime(3);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage2(6, true) == PistonClimbSubsystem.stageState.IN_PROG);
        PistonClimbSubsystem.getInstance().setTime(7);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage2(6, true) == PistonClimbSubsystem.stageState.COMPLETE);

        PistonClimbSubsystem.getInstance().setRange(0); //Tests what happens if robot is in theory rolling successfully onto top of platform
        PistonClimbSubsystem.getInstance().setTime(3);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage5(6, true) == PistonClimbSubsystem.stageState.IN_PROG);
        PistonClimbSubsystem.getInstance().setTime(7);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage5(6, true) == PistonClimbSubsystem.stageState.COMPLETE);

        PistonClimbSubsystem.getInstance().setDetectionTime(0);
        PistonClimbSubsystem.getInstance().setRange(800); //Tests what happens if robot has not reached platform yet while rolling towards platform
        PistonClimbSubsystem.getInstance().setTime(3);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage2(6, true) == PistonClimbSubsystem.stageState.IN_PROG);
        PistonClimbSubsystem.getInstance().setTime(7);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage2(6, true) == PistonClimbSubsystem.stageState.FAILED);

        PistonClimbSubsystem.getInstance().setRange(800); //Tests what happens if robot is not on platform yet while rolling onto platform
        PistonClimbSubsystem.getInstance().setTime(3);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage5(6, true) == PistonClimbSubsystem.stageState.IN_PROG);
        PistonClimbSubsystem.getInstance().setTime(7);
        Assert.assertTrue(PistonClimbSubsystem.getInstance().climbStage5(6, true) == PistonClimbSubsystem.stageState.FAILED);
        PistonClimbSubsystem.getInstance().setPistonClimbNotTesting(false);
    }
}