package com.openvins.android.component

import com.openvins.android.engine.Route
import com.openvins.android.engine.Smoother
import com.openvins.android.engine.Utils
import com.openvins.android.models.Pose
import com.openvins.android.models.RadiationField
import com.openvins.android.models.Result
import com.openvins.android.models.SE3
import com.openvins.android.models.SO3
import com.openvins.android.models.Trajectory
import glm_.vec3.Vec3d
import kotlin.math.max

class TrajectoryRevisitor {
    private val _config: Map<String, Any> = emptyMap()
    private var _revisitMileage: Double = 1.5
    private var _maxDistance: Double = 0.5
    private var _impactThreshold: Double = 0.5
    private var _absPoseErrRotFro: Double = 0.1
    private var _pointDistance: Double = 0.5

    private var _smoothingSigma: Double = 1.0
    private var _sliceRemind: Int = 300
    private var _minTrajectoryLength: Int = 20

    constructor() {}

    fun setConfig(config: Map<String, Any>) {
        _config.plus(config)
        _revisitMileage = _config.getOrElse("revisitMileage") { 1.5 } as Double
        _maxDistance = _config.getOrElse("maxDistance") { 0.5 } as Double
        _impactThreshold = _config.getOrElse("impactThreshold") { 0.5 } as Double

        _smoothingSigma = _config.getOrElse("smoothingSigma") { 1.0 } as Double
        _sliceRemind = _config.getOrElse("sliceRemind") { 200 } as Int
        _minTrajectoryLength = _config.getOrElse("minTrajectoryLength") { 20 } as Int
    }

    fun searchRevisitTrajectory(
        translations: FloatArray,
        quaternions: FloatArray,
        translation: DoubleArray,
        quaternion: DoubleArray,
    ): Result {
        val dTranslations = DoubleArray(translations.size)
        for (i in 0 until translations.size) {
            dTranslations[i] = translations[i].toDouble()
        }
        val dQuaternions = DoubleArray(quaternions.size)
        for (i in 0 until quaternions.size) {
            dQuaternions[i] = quaternions[i].toDouble()
        }
        return searchRevisitTrajectory(
            dTranslations,
            dQuaternions,
            translation,
            quaternion,
        )
    }

    fun searchRevisitTrajectory(
        translations: DoubleArray,
        quaternions: DoubleArray,
        translation: DoubleArray,
        quaternion: DoubleArray,
    ): Result {
        var trajectory =
            Trajectory.create(translations, quaternions) + Pose.create(translation, quaternion)
        if (trajectory.size() <= _minTrajectoryLength) {
            return Result(false, 0.0, 0.0)
        }

        var outerTrajectory = buildOuterTrajectory(trajectory)
        trajectory = smoothTrajectory(trajectory)
        outerTrajectory = smoothTrajectory(outerTrajectory)

        val tmpMileage = Route.calculateMileage(trajectory.points)
        val tmpOuterMileage = Route.calculateMileage(outerTrajectory.points)
        val threshold = _revisitMileage + _maxDistance * 2.0
        if (tmpMileage < threshold || tmpOuterMileage < threshold) {
            return Result(false, 1.0, 0.0)
        }

        val tripleRange0 =
            Route.getMinimumSubPathReversed(trajectory.points, _revisitMileage, _maxDistance)
        if (tripleRange0.first < 0 || tripleRange0.third < 0) {
            return Result(false, 2.0, 0.0)
        }
        val queryTrajectory = trajectory.slice(tripleRange0.first, tripleRange0.second)
        trajectory = trajectory.slice(0, tripleRange0.third)

        val tripleRange1 =
            Route.getMinimumSubPathReversed(outerTrajectory.points, _revisitMileage, _maxDistance)
        if (tripleRange1.first < 0 || tripleRange1.third < 0) {
            return Result(false, 3.0, 0.0)
        }
        val queryOuterTrajectory = outerTrajectory.slice(tripleRange1.first, tripleRange1.second)
        outerTrajectory = outerTrajectory.slice(0, tripleRange1.third)

        val usedTrajectory = sliceTrajectory(trajectory)
        val usedOuterTrajectory = sliceTrajectory(outerTrajectory)

        val radiationField = RadiationField(usedTrajectory, _maxDistance)
        val impact = radiationField.calculateRadiationImpact(queryTrajectory)
        val outerRadiationField = RadiationField(usedOuterTrajectory, _maxDistance)
        val outerImpact = outerRadiationField.calculateRadiationImpact(queryOuterTrajectory)
        return Result(
            impact >= _impactThreshold && outerImpact >= _impactThreshold,
            impact, outerImpact
        )
    }

    fun queryVisitedPoses(
        candidateTranslations: List<DoubleArray>,
        candidateQuaternions: List<DoubleArray>,
        queryTranslation: DoubleArray,
        queryQuaternion: DoubleArray,
    ): Result {
        if (candidateTranslations.size != candidateQuaternions.size) {
            return Result(false)
        }
        val invQueryPose = SO3.create(queryQuaternion).inverse()
        for (i in candidateTranslations.indices) {
            val candidatePose = SO3.create(candidateQuaternions[i])
            val resultQuat = (invQueryPose * candidatePose).toQuat()
            val absPoseErrRotFro = Vec3d(resultQuat.x, resultQuat.y, resultQuat.z).length()
            val pointDistance =
                Utils.distancePointToPoint3d(candidateTranslations[i], queryTranslation)
            if (absPoseErrRotFro < _absPoseErrRotFro && pointDistance < _pointDistance) {
                return Result(
                    true,
                    absPoseErrRotFro = absPoseErrRotFro,
                    pointDistance = pointDistance
                )
            }
        }
        return Result(false)
    }

    private fun buildOuterTrajectory(trajectory: Trajectory): Trajectory {
        val initPoint = Vec3d(1.0, 0.0, 0.0)
        val n = trajectory.size()
        val outerPositions = mutableListOf<Vec3d>()
        for (i in 0 until n) {
            val position = SE3.create(trajectory[i]) * initPoint
            outerPositions.add(position)
        }
        return Trajectory(translations = outerPositions, quaternions = trajectory.quaternions)
    }

    private fun smoothTrajectory(trajectory: Trajectory): Trajectory {
        val smoothPoints = Smoother.gaussianSmooth(
            trajectory.points, _smoothingSigma
        )
        return Trajectory(translations = smoothPoints, quaternions = trajectory.quaternions)
    }

    private fun sliceTrajectory(trajectory: Trajectory): Trajectory {
        val step = max(1, trajectory.size() / _sliceRemind)
        val slicedTranslations =
            trajectory.translations.slice(0 until trajectory.translations.size step step)
        val slicedQuaternions =
            trajectory.quaternions.slice(0 until trajectory.quaternions.size step step)
        return Trajectory(slicedTranslations, slicedQuaternions)
    }
}