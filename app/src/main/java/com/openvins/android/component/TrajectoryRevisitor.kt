package com.openvins.android.component

import com.openvins.android.engine.Route
import com.openvins.android.engine.Smoother
import com.openvins.android.models.Pose
import com.openvins.android.models.RadiationField
import com.openvins.android.models.SE3
import com.openvins.android.models.Trajectory
import glm_.vec3.Vec3d
import kotlin.math.max

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
        var trajectory =
            Trajectory.create(translations, quaternions) + Pose.create(translation, quaternion)

        var outerTrajectory = buildOuterTrajectory(trajectory)
        trajectory = smoothTrajectory(trajectory)
        outerTrajectory = smoothTrajectory(outerTrajectory)

        val vMileage = _config.getOrElse("revisitMileage") { 1.5 } as Double
        val vMaxDistance = _config.getOrElse("maxDistance") { 0.5 } as Double

        val tmpMileage = Route.calculateMileage(trajectory.points)
        val tmpOuterMileage = Route.calculateMileage(outerTrajectory.points)
        val threshold = vMileage + vMaxDistance * 2.0
        if (tmpMileage < threshold || tmpOuterMileage < threshold) {
            return false
        }

        val tripleRange0 =
            Route.getMinimumSubPathReversed(trajectory.points, vMileage, vMaxDistance)
        if (tripleRange0.first < 0 || tripleRange0.third < 0) {
            return false
        }
        val queryTrajectory = trajectory.slice(tripleRange0.first, tripleRange0.second)
        trajectory = trajectory.slice(0, tripleRange0.third)

        val tripleRange1 =
            Route.getMinimumSubPathReversed(outerTrajectory.points, vMileage, vMaxDistance)
        if (tripleRange1.first < 0 || tripleRange1.third < 0) {
            return false
        }
        val queryOuterTrajectory = outerTrajectory.slice(tripleRange1.first, tripleRange1.second)
        outerTrajectory = outerTrajectory.slice(0, tripleRange1.third)

        val usedTrajectory = sliceTrajectory(trajectory)
        val usedOuterTrajectory = sliceTrajectory(outerTrajectory)
        val range0 =
            Route.getMaximumSubPath(queryTrajectory.points, usedTrajectory.points, vMaxDistance)
        if (range0.first < 0) {
            return false
        }
        val subTrajectory = usedTrajectory.slice(range0.first, range0.second)
        val range1 = Route.getMaximumSubPath(
            queryOuterTrajectory.points,
            usedOuterTrajectory.points,
            vMaxDistance
        )
        if (range1.first < 0) {
            return false
        }
        val subOuterTrajectory = usedOuterTrajectory.slice(range1.first, range1.second)

        val radiationField = RadiationField(subTrajectory, vMaxDistance)
        val impact = radiationField.calculateRadiationImpact(queryTrajectory)
        val outerRadiationField = RadiationField(subOuterTrajectory, vMaxDistance)
        val outerImpact = outerRadiationField.calculateRadiationImpact(queryOuterTrajectory)
        val impactThreshold = _config.getOrElse("impactThreshold") { 0.5 } as Double
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

    private fun sliceTrajectory(trajectory: Trajectory): Trajectory {
        val sliceRemind = _config.getOrElse("sliceRemind") { 100 } as Int
        val step = max(1, trajectory.size() / sliceRemind)
        val slicedTranslations =
            trajectory.translations.slice(0 until trajectory.translations.size step step)
        val slicedQuaternions =
            trajectory.quaternions.slice(0 until trajectory.quaternions.size step step)
        return Trajectory(slicedTranslations, slicedQuaternions)
    }
}