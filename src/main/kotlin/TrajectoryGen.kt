import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan


object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(52.0, 52.0, 0.0, 250.0.toRadians, 250.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 12.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private const val halfMat = 12.0

    private val startPose = Pose2d(-halfMat * 6 + 7.0, -halfMat * 3, (-90.0).toRadians)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)


        val armLength = 7.0 // in inch
        val alpha = 35.0
        val beta = (90.0 - alpha) / 2.0

        // add calculate equations here
        val eg = halfMat
        val cg = eg / cos(alpha.toRadians)
        val ag = armLength
        val ac = cg - ag
        val ao = ac * tan(beta.toRadians)
        val bo = ao
        val fo = ao
        val ax = armLength * sin(alpha.toRadians)
        val gx = armLength * cos(alpha.toRadians)
        val bc = ac
        val ce = eg * tan(alpha.toRadians)
        val be = ce - bc
        val co = ac / cos(beta.toRadians)
        val cVx = -eg * 2 + ce
        val cf = co - fo
        val fh = cf * bo / co
        val ch = cf * bc / co


        // update below 6 variable values
        val Ax = ax - eg * 2
        val Ay = -eg * 2 - gx
        val Fx = cVx - ch
        val Fy = fh - eg * 3
        val Bx = be - eg * 2
        val By = -eg * 3

        print("start    ")
        print(" / co = ")
        print( co)
        print(" / bc = ")
        print( bc)
        print(" / ce = ")
        print( ce)

        print(" / Bx = ")
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

        val bB = Pose2d(Bx, By, (-90.0).toRadians)
        val fF = Pose2d(Fx, Fy, (-90.0/2.0 + alpha/2.0 - 45.0).toRadians)
        val aA = Pose2d(Ax +0.0, Ay+0.0, (-90 + alpha).toRadians)



        // Small Example Routine
        builder1
            .strafeLeft(24.0)
            .splineToSplineHeading(bB, 0.0)
            .splineToSplineHeading(fF, (90.0 + alpha).toRadians / 2.0)
            .splineToSplineHeading(aA, (90.0 +  alpha).toRadians);

        list.add(builder1.build())
/*

       val bB = Pose2d(Bx, By, (-90.0).toRadians)
        val fF = Pose2d(Fx, Fy, (-90.0 + alpha / 2.0).toRadians)
        val aA = Pose2d(Ax +0.0, Ay+0.0, (-90 + alpha).toRadians)

        .splineToSplineHeading(bB, 0.0)
            .splineToSplineHeading(fF, (90.0 + alpha).toRadians / 2.0)
            .splineToSplineHeading(aA, (90.0 +  alpha).toRadians);

  builder1
            .strafeLeft(24.0)
            .splineTo(Vector2d(bB.x, bB.y), 0.0)
            .splineTo(Vector2d(fF.x, fF.y), (90.0 + alpha).toRadians / 2.0)
            .splineTo(Vector2d(aA.x, aA.y), (90.0 + alpha).toRadians);

        list.add(builder1.build())

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

        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(-72.0 + 7.0, -36.0), 14.0, 15.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))
