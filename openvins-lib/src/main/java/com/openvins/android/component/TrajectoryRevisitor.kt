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
import kotlin.math.sqrt

class TrajectoryRevisitor {
    private var _config: Map<String, Any> = emptyMap()
    private var _revisitMileage: Double = 1.5
    private var _maxDistance: Double = 0.5
    private var _impactThreshold: Double = 0.35
    private var _absPoseErrRotFro: Double = 0.2
    private var _pointDistance: Double = 0.5
    private var _sameSideCosThreshold: Double = 0.0
    private var _shiftingMileage: Double = 1.0

    private var _smoothingSigma: Double = 1.0
    private var _sliceRemind: Int = 300
    private var _minTrajectoryLength: Int = 20

    private val _trajectoryShifting = mutableListOf<Pose>()

    constructor() {}

    fun setConfig(config: Map<String, Any>) {
        _config = config.toMap()
        _revisitMileage = doubleConfig("revisitMileage", 1.5)
        _maxDistance = doubleConfig("maxDistance", 0.5)
        _impactThreshold = doubleConfig("impactThreshold", 0.5)
        _absPoseErrRotFro = doubleConfig("absPoseErrRotFro", 0.2)
        _pointDistance = doubleConfig("pointDistance", 0.5)
        _sameSideCosThreshold = doubleConfig("sameSideCosThreshold", 0.0)
        _shiftingMileage = doubleConfig("shiftingMileage", 1.0)

        _smoothingSigma = doubleConfig("smoothingSigma", 1.0)
        _sliceRemind = intConfig("sliceRemind", 200)
        _minTrajectoryLength = intConfig("minTrajectoryLength", 20)
    }

    private fun doubleConfig(key: String, defaultValue: Double): Double {
        return (_config[key] as? Number)?.toDouble() ?: defaultValue
    }

    private fun intConfig(key: String, defaultValue: Int): Int {
        return (_config[key] as? Number)?.toInt() ?: defaultValue
    }

    private fun f2d(v: FloatArray): DoubleArray {
        val d = DoubleArray(v.size)
        for ((i, element) in v.withIndex()) {
            d[i] = element.toDouble()
        }
        return d
    }

    fun searchRevisitTrajectory(
        translations: FloatArray,
        quaternions: FloatArray,
        translation: DoubleArray,
        quaternion: DoubleArray,
    ): Result {
        return searchRevisitTrajectory(
            f2d(translations),
            f2d(quaternions),
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
            return Result()
        }

        var outerTrajectory = buildOuterTrajectory(trajectory)
        trajectory = smoothTrajectory(trajectory)
        outerTrajectory = smoothTrajectory(outerTrajectory)

        val tmpMileage = Route.calculateMileage(trajectory.points)
        val tmpOuterMileage = Route.calculateMileage(outerTrajectory.points)
        val threshold = _revisitMileage + _maxDistance * 2.0
        if (tmpMileage < threshold || tmpOuterMileage < threshold) {
            return Result(impact = 1.0)
        }

        val tripleRange0 =
            Route.getMinimumSubPathReversed(trajectory.points, _revisitMileage, _maxDistance)
        if (tripleRange0.first < 0 || tripleRange0.third < 0) {
            return Result(impact = 2.0)
        }
        val queryTrajectory = trajectory.slice(tripleRange0.first, tripleRange0.second)
        trajectory = trajectory.slice(0, tripleRange0.third)

        val tripleRange1 =
            Route.getMinimumSubPathReversed(outerTrajectory.points, _revisitMileage, _maxDistance)
        if (tripleRange1.first < 0 || tripleRange1.third < 0) {
            return Result(impact = 3.0)
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
            isDuplicated = impact >= _impactThreshold && outerImpact >= _impactThreshold,
            impact = impact, outerImpact = outerImpact
        )
    }

    fun queryVisitedPoses(
        candidateTranslations: List<DoubleArray>,
        candidateQuaternions: List<DoubleArray>,
        queryTranslation: DoubleArray,
        queryQuaternion: DoubleArray,
    ): Result {
        if (candidateTranslations.size != candidateQuaternions.size) {
            return Result()
        }
        val invQueryPose = SO3.create(queryQuaternion).inverse()
        val queryDirection = horizontalCameraDirection(queryQuaternion)
        var bestRotationError = 0.0
        var bestPointDistance = 1000.0
        var foundSameSide = false
        for (i in candidateTranslations.indices) {
            val candidatePose = SO3.create(candidateQuaternions[i])
            val resultQuat = (invQueryPose * candidatePose).toQuat()
            val absPoseErrRotFro = Vec3d(resultQuat.x, resultQuat.y, resultQuat.z).length()
            val pointDistance =
                Utils.distancePointToPoint(candidateTranslations[i], queryTranslation)

            // 猪圈位于过道两侧：站位相近但镜头朝向相反时，应视为拍摄不同侧猪圈。
            val candidateDirection = horizontalCameraDirection(candidateQuaternions[i])
            val sameSide = horizontalDot(queryDirection, candidateDirection) > _sameSideCosThreshold
            if (!sameSide) {
                continue
            }

            // 同侧候选中选择距离最近的一次，避免历史数组顺序影响最终判断。
            foundSameSide = true
            if (pointDistance < bestPointDistance) {
                bestRotationError = absPoseErrRotFro
                bestPointDistance = pointDistance
            }
        }
        return Result(
            isRetrieve = foundSameSide && bestPointDistance < _pointDistance,
            absPoseErrRotFro = bestRotationError,
            pointDistance = bestPointDistance,
            isSameSide = foundSameSide,
        )
    }

    /**
     * 使用相机局部 X 轴作为拍摄方向，并投影到水平面。
     * 与轨迹外轮廓计算保持同一坐标约定，忽略手持俯仰和横滚带来的影响。
     */
    private fun horizontalCameraDirection(quaternion: DoubleArray): DoubleArray {
        val direction = SO3.create(quaternion).matrix * Vec3d(1.0, 0.0, 0.0)
        val norm = sqrt(direction.x * direction.x + direction.y * direction.y)
        if (norm < 1e-6) {
            return doubleArrayOf(0.0, 0.0)
        }
        return doubleArrayOf(direction.x / norm, direction.y / norm)
    }

    private fun horizontalDot(first: DoubleArray, second: DoubleArray): Double {
        if ((first[0] == 0.0 && first[1] == 0.0) ||
            (second[0] == 0.0 && second[1] == 0.0)
        ) {
            return -1.0
        }
        return first[0] * second[0] + first[1] * second[1]
    }

    fun captureRevisited(
        translations: FloatArray,
        quaternions: FloatArray,
        translation: DoubleArray,
        quaternion: DoubleArray,
        candidateTranslations: List<DoubleArray>,
        candidateQuaternions: List<DoubleArray>,
        queryTranslation: DoubleArray,
        queryQuaternion: DoubleArray,
    ): Result {
        return captureRevisited(
            f2d(translations),
            f2d(quaternions),
            translation,
            quaternion,
            candidateTranslations,
            candidateQuaternions,
            queryTranslation,
            queryQuaternion,
        )
    }

    fun captureRevisited(
        translations: DoubleArray,
        quaternions: DoubleArray,
        translation: DoubleArray,
        quaternion: DoubleArray,
        candidateTranslations: List<DoubleArray>,
        candidateQuaternions: List<DoubleArray>,
        queryTranslation: DoubleArray,
        queryQuaternion: DoubleArray,
    ): Result {
        val result0 = queryVisitedPoses(
            candidateTranslations,
            candidateQuaternions,
            queryTranslation,
            queryQuaternion
        )
        val result1 = searchRevisitTrajectory(translations, quaternions, translation, quaternion)
        // 路径重合只能作为同侧拍摄的辅助条件，避免站在过道同一位置转向另一侧时误报。
        val duplicatedOnSameSide = result0.isSameSide && result1.isDuplicated
        val result = Result(
            result0.isRetrieve || duplicatedOnSameSide,
            isRetrieve = result0.isRetrieve,
            isDuplicated = duplicatedOnSameSide,
            impact = result1.impact,
            outerImpact = result1.outerImpact,
            absPoseErrRotFro = result0.absPoseErrRotFro,
            pointDistance = result0.pointDistance,
            isSameSide = result0.isSameSide,
        )
        return result
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

    fun shiftingTrajectory(
        translation: DoubleArray,
        quaternion: DoubleArray,
    ): Boolean {
        _trajectoryShifting.add(Pose.create(
            translation = translation, quaternion = quaternion
        ))
        val lastTT = System.nanoTime() - 5e8
        val tmpTrajectory = _trajectoryShifting.filter { it.timestamp >= lastTT }
        val mileageInSecond = Route.calculateMileage(tmpTrajectory.map { it.position })
        _trajectoryShifting.clear()
        _trajectoryShifting.addAll(tmpTrajectory)
        return mileageInSecond > _shiftingMileage
    }

    fun reset(){
        _trajectoryShifting.clear()
    }
}
