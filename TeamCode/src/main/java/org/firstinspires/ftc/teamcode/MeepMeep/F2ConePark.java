//package com.example.meepmeeptesting.soniatest;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.noahbres.meepmeep.roadrunner.DriveShim;
//import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
//
//public class F2ConePark implements MeepMeepPath {
//
//	//red start
//	Pose2d F2StartPos = new Pose2d( -34,-72 + (robotLength/2), Math.toRadians(270));
//
//	//red end
////	Pose2d F2EndPos1 = new Pose2d(-48 - (robotWidth/2), -48 + (robotLength/2), 90);
////	Pose2d F2EndPos2 = new Pose2d(-24 - (robotWidth/2), -48 + (robotLength/2), 90);
////	Pose2d F2EndPos3 = new Pose2d( 0 - (robotWidth/2), -48 + (robotLength/2), 90);
//
//	@Override
//	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
//		return drive.trajectorySequenceBuilder(F2StartPos)
//				//go to drop pos for cone 1
//				//.lineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)))
//				.back(54)
//				.turn(Math.toRadians(-45))
//				.waitSeconds(1)
//
//				//collect and drop cone 2
//				.splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//				.waitSeconds(1)
//				.splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//				.waitSeconds(1)
//
//				//collect and drop cone 3
//				.splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//				.waitSeconds(1)
//				.splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//				.waitSeconds(1)
//
//				//collect and drop cone 4
//				.splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//				.waitSeconds(1)
//				.splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//				.waitSeconds(1)
//
//				//collect and drop cone 5
//				.splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//				.waitSeconds(1)
//				.splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//				.waitSeconds(1)
//
//				//collect and drop cone 6
//				.splineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(180)), Math.toRadians(0))
//				.waitSeconds(1)
//				.splineToLinearHeading(new Pose2d(-34, -8, Math.toRadians(-140)), Math.toRadians(180))
//				.waitSeconds(1)
//
//				.build( );
//	}
//}
