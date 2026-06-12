package com.openvins.android.component

import com.openvins.android.engine.Route
import com.openvins.android.engine.Smoother
import com.openvins.android.models.Pose
import com.openvins.android.models.RadiationField
import com.openvins.android.models.SE3
import com.openvins.android.models.Trajectory
import glm_.vec3.Vec3d

class TrajectoryRevisitor {
    private val _config: Map<String, Any> = emptyMap()

    constructor() {}

    fun setConfig(config: Map<String, Any>) {
        _config.plus(config)
    }

    fun searchRevisitTrajectory(
        translations: DoubleArray,
        quaternions: DoubleArray,
        translation: DoubleArray,
        quaternion: DoubleArray,
    ): Boolean {
        var trajectory = (
                Trajectory.create(translations, quaternions, order = "xyzs") +
                        Pose.create(translation, quaternion, order = "xyzs")
                )
        var outerTrajectory = buildOuterTrajectory(trajectory)
        trajectory = smoothTrajectory(trajectory)
        outerTrajectory = smoothTrajectory(outerTrajectory)

        val vMileage = _config.getOrElse("revisitMileage") { 1.5 } as Double
        val queryTrajectory =
            sliceTrajectory(Route.getMinimumSubPath(trajectory, vMileage, reversed = true))
        val queryOuterTrajectory =
            sliceTrajectory(Route.getMinimumSubPath(outerTrajectory, vMileage, reversed = true))
        val usedTrajectory = sliceTrajectory(trajectory)
        val usedOuterTrajectory = sliceTrajectory(outerTrajectory)

        val maxDistance = _config.getOrElse("maxDistance") { 0.5 } as Double
        val radiationField = RadiationField(usedTrajectory, maxDistance)
        val impact = radiationField.calculateRadiationImpact(queryTrajectory)
        val outerRadiationField = RadiationField(usedOuterTrajectory, maxDistance)
        val outerImpact = outerRadiationField.calculateRadiationImpact(queryOuterTrajectory)
        val impactThreshold = _config.getOrElse("impactThreshold") { 0.2 } as Double
        return impact >= impactThreshold && outerImpact >= impactThreshold
    }

    private fun buildOuterTrajectory(trajectory: Trajectory): Trajectory {
        val initPoint = Vec3d(1.0, 0.0, 0.0)
        val n = trajectory.size()
        val outerPositions = mutableListOf<Vec3d>()
        for (i in 0 until n) {
            val position = SE3.create(trajectory[i]) * initPoint
            outerPositions.add(position)
        }
        return trajectory.copy(translations = outerPositions)
    }

    private fun smoothTrajectory(trajectory: Trajectory): Trajectory {
        val smoothPoints = Smoother.gaussianSmooth(
            trajectory.points,
            _config.getOrElse("smoothingSigma") { 1.0 } as Double)
        return trajectory.copy(translations = smoothPoints)
    }

    private fun sliceTrajectory(trajectory: Trajectory, step: Int = 5): Trajectory {
        val interval = _config.getOrElse("sliceInterval") { step } as Int
        val slicedTranslations =
            trajectory.translations.slice(0 until trajectory.translations.size step interval)
        val slicedQuaternions =
            trajectory.quaternions.slice(0 until trajectory.quaternions.size step interval)
        return Trajectory(slicedTranslations, slicedQuaternions)
    }
}