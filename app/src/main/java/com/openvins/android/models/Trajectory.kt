package com.openvins.android.models

import glm_.quat.QuatD
import glm_.vec3.Vec3d

data class Result(
    val isRevisit: Boolean,
    val impact: Double,
    val outerImpact: Double,
)

data class Pose(val translation: Vec3d, val quaternion: QuatD) {
    val position: Vec3d = translation
    val point: Vec3d = translation

    companion object {
        fun create(
            translation: DoubleArray,
            quaternion: DoubleArray,
            order: String = "sxyz"
        ): Pose {
            val t = Vec3d(translation[0], translation[1], translation[2])
            val q = if (order == "sxyz") {
                QuatD(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            } else {
                QuatD(quaternion[3], quaternion[0], quaternion[1], quaternion[2])
            }
            return Pose(t, q)
        }
    }
}

data class Trajectory(
    val translations: List<Vec3d>,
    val quaternions: List<QuatD>,
) {
    val positions: List<Vec3d> = translations
    val points: List<Vec3d> = translations

    companion object {
        fun empty(): Trajectory {
            return Trajectory(emptyList(), emptyList())
        }

        fun create(
            translations: DoubleArray,
            quaternions: DoubleArray,
            order: String = "sxyz"
        ): Trajectory {
            val n = translations.size / 3
            assert(quaternions.size / 4 == n) { "quaternions != translations" }
            val pts = mutableListOf<Vec3d>()
            val quat = mutableListOf<QuatD>()
            for (i in 0 until n) {
                pts.add(
                    Vec3d(
                        translations[3 * i],
                        translations[3 * i + 1],
                        translations[3 * i + 2]
                    )
                )
                if (order == "sxyz") {
                    quat.add(
                        QuatD(
                            quaternions[4 * i],
                            quaternions[4 * i + 1],
                            quaternions[4 * i + 2],
                            quaternions[4 * i + 3]
                        )
                    )
                } else {
                    quat.add(
                        QuatD(
                            quaternions[4 * i + 3],
                            quaternions[4 * i],
                            quaternions[4 * i + 1],
                            quaternions[4 * i + 2]
                        )
                    )
                }
            }
            return Trajectory(pts.toList(), quat.toList())
        }
    }

    operator fun get(index: Int): Pose {
        return Pose(translations[index], quaternions[index])
    }

    operator fun plus(pose: Pose): Trajectory {
        val newTranslations = translations + pose.translation
        val newQuaternions = quaternions + pose.quaternion
        return Trajectory(newTranslations, newQuaternions)
    }

    fun size(): Int {
        return translations.size
    }

    fun slice(start: Int, end: Int): Trajectory {
        return Trajectory(
            translations = positions.subList(start, end),
            quaternions = quaternions.subList(start, end),
        )
    }

}