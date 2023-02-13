import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
<<<<<<< HEAD
import javafx.scene.chart.Axis
import java.lang.Thread.sleep
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan
=======

>>>>>>> 250e05e9011bbf3470d2748bf55587f60679725d

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


        val armLength = 7.0 // in inch
        val alpha = 30.0
        val beta = alpha / 2.0

        // add calculate equations here
        val eg = 12.0
        val cg = eg / cos(alpha.toRadians)
        val ag = armLength
        val ac = cg - ag
        val ao = ac * tan(beta.toRadians)
        val bo = ao
        val fo = ao
        val ax = 7 * sin(alpha.toRadians)
        val gx = 7 * cos(alpha.toRadians)
        val bc = ac
        val ce = eg * tan(alpha.toRadians)
        val be = ce - bc
        val co = ac / cos(beta.toRadians)
        val cVx = -24 + ce
        val cf = co - fo
        val fh = cf * bo / co
        val ch = cf * bc / co





        // update below 6 variable values
        val Ax = ax - 24
        val Ay = -24 - gx
        val Fx = cVx - ch
        val Fy = fh - 36
        val Bx = be - 24
        val By = -36.0

        print("Bx = ")
        print( Bx)
        print(" / By = ")
        print( By)
        print(" / Fx = ")
        print( Fx)
        print(" / Fy = ")
        print( Fy)
        print(" / Ax = ")
        print( Ax)
        print(" / Ay = ")
        print( Ay)
        print("  / ")

        val aA = Pose2d(Ax, Ay, (-90.0).toRadians)
        val fF = Pose2d(Fx, Fy, (-90.0 + alpha / 2.0).toRadians)
        val bB = Pose2d(Bx, By, -(90 - alpha).toRadians)


        // Small Example Routine
        builder1
            .strafeLeft(24.0)
<<<<<<< HEAD

            .splineToSplineHeading(bB, 0.0)
            .splineToSplineHeading(fF, (180.0 - alpha).toRadians / 2.0)
            //.splineToSplineHeading(aA, (180.0 - alpha).toRadians);
        list.add(builder1.build())

        /*

        for (i in 0 until 4) {
=======
            .splineToSplineHeading(Pose2d(-24.0, -36.0, -80.0.toRadians), 0.0)
            .splineToSplineHeading(Pose2d(-24.0 + 5, -33.0, -70.0.toRadians), 70.0.toRadians)
            //.splineTo(Vector2d(-48.0 + 6, -36.0 - 6), -45.0.toRadians)
            //.splineTo(Vector2d(-24.0  + 2, -36.0 + 5.6), 90.0.toRadians + 20.0.toRadians)
            .splineToLinearHeading(Pose2d(-24.0  + 2, -36.0 + 5.6, -70.0.toRadians), 70.0.toRadians);
            //.lineToConstantHeading(Vector2d(-24.0  + 2.0,-36.0 + 5.6));
        list.add(builder1.build())
/*
        for (i in 0 until 2) {
>>>>>>> 250e05e9011bbf3470d2748bf55587f60679725d
            val start2Pose = Pose2d(-24.0 + 2, -36.0 + 5.6, -90.0.toRadians + 20.0.toRadians)
            val builder2 = TrajectoryBuilder(start2Pose, startPose.heading, combinedConstraints)
            builder2
                .lineToLinearHeading(Pose2d(-12.0, -62.0, -90.0.toRadians));


            val start3Pose = Pose2d(-12.0, -62.0, -90.0.toRadians)
            val builder3 = TrajectoryBuilder(start3Pose, startPose.heading, combinedConstraints)
            builder3
                .lineToLinearHeading(start2Pose);

           // list.add(builder2.build())
            //list.add(builder3.build())
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

        //list.add(builder2.build())
        //list.add(builder3.build())

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
<<<<<<< HEAD

         */
=======
>>>>>>> 250e05e9011bbf3470d2748bf55587f60679725d
        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 14.0, 15.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))
