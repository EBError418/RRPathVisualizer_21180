import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints


object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(52.0, 52.0, 0.0, 250.0.toRadians, 250.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 12.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private val startPose = Pose2d(-72.0 + 7.0, -36.0, -90.0.toRadians)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)

//        builder1.splineTo(Vector2d(-48.0, -36.0), -45.0.toRadians)


        // Small Example Routine
        builder1
            .strafeLeft(24.0)
            .splineToSplineHeading(Pose2d(-24.0, -36.0, -80.0.toRadians), 0.0)
            .splineToSplineHeading(Pose2d(-24.0 + 5, -33.0, -70.0.toRadians), 70.0.toRadians)
            //.splineTo(Vector2d(-48.0 + 6, -36.0 - 6), -45.0.toRadians)
            //.splineTo(Vector2d(-24.0  + 2, -36.0 + 5.6), 90.0.toRadians + 20.0.toRadians)
            .splineToLinearHeading(Pose2d(-24.0  + 2, -36.0 + 5.6, -70.0.toRadians), 70.0.toRadians);
            //.lineToConstantHeading(Vector2d(-24.0  + 2.0,-36.0 + 5.6));
        list.add(builder1.build())
/*
        for (i in 0 until 2) {
            val start2Pose = Pose2d(-24.0 + 2, -36.0 + 5.6, -90.0.toRadians + 20.0.toRadians)
            val builder2 = TrajectoryBuilder(start2Pose, startPose.heading, combinedConstraints)
            builder2
                .lineToLinearHeading(Pose2d(-12.0, -62.0, -90.0.toRadians));

            sleep(500);
            val start3Pose = Pose2d(-12.0, -62.0, -90.0.toRadians)
            val builder3 = TrajectoryBuilder(start3Pose, startPose.heading, combinedConstraints)
            builder3
                .lineToLinearHeading(start2Pose);
            sleep(500);

            list.add(builder2.build())
            list.add(builder3.build())

        }

 */

        // high junction for 5th cone
        /*
        val start2Pose = Pose2d(-24.0 + 2, -36.0 + 5.6, -90.0.toRadians + 20.0.toRadians)
        val builder2 = TrajectoryBuilder(start2Pose, startPose.heading, combinedConstraints)
        builder2
            .lineToLinearHeading(Pose2d(-12.0, -62.0, -90.0.toRadians));
        sleep(500);

        val start3Pose = Pose2d(-12.0, -62.0, -90.0.toRadians)
        val hjPose = Pose2d(-2.0, -36.0+5.6, -110.0.toRadians)
        val builder3 = TrajectoryBuilder(start3Pose, startPose.heading, combinedConstraints)
        builder3
            .lineToLinearHeading(hjPose);
        sleep(500);

        list.add(builder2.build())
        list.add(builder3.build())

         */
        //Pose2d(-2 * Params.HALF_MAT + armX, -2 * Params.HALF_MAT + armY, dropOffAngle);
        val start2Pose = Pose2d(-24.0 + 2, -24.0 - 5.6, -90.0.toRadians + 20.0.toRadians)
        // parking
        val p2 = Pose2d(-2.5 * 12, -3 * 12.0, -180.0.toRadians)
        var p3 = Pose2d(-36.0, - 36 - 10.0, -180.0.toRadians)
        val parkPose = Pose2d(-36.0, -36.0 - 24, -180.0.toRadians)

        /*
                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(poseParkingEnd1, parkingEndTangent)
                .splineToSplineHeading(poseParking, parkingEndTangent)
                .build();
        drive.followTrajectory(traj1);
         */
        val builder4 = TrajectoryBuilder(start2Pose, startPose.heading, combinedConstraints)
        builder4
            .splineToSplineHeading(p2, -180.0.toRadians)
            .splineToSplineHeading(p3, -90.0.toRadians)
            .splineToSplineHeading(parkPose, -90.0.toRadians)

        //list.add(builder4.build())
        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))
