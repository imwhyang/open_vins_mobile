package com.openvins.android.models

import glm_.mat3x3.Mat3d
import glm_.quat.QuatD
import glm_.vec3.Vec3d


class SO3(val matrix: Mat3d) {
    companion object {
        fun identity(): SO3 {
            return SO3(Mat3d(1.0))
        }

        fun create(quaternion: QuatD): SO3 {
            val norm = quaternion.normalize()
            val x = norm.x
            val y = norm.y
            val z = norm.z
            val w = norm.w
            val mat = Mat3d(
                1.0 - 2  * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                2 * (x * y + z * w), 1.0 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                2 * (x * z - y * w), 2 * (y * z + x * w), 1.0 - 2 * (x * x + y * y)
            )
            return SO3(mat)
        }
        fun create(vector: DoubleArray, order: String = "sxyz"): SO3 {
            return if (order == "sxyz") {
                create(QuatD(vector[0], vector[1], vector[2], vector[3]))
            } else {
                create(QuatD(vector[3], vector[0], vector[1], vector[2]))
            }
        }
    }
    operator fun times(other: SO3): SO3 {
        return SO3(this.matrix * other.matrix)
    }
    fun inverse(): SO3 {
        return SO3(this.matrix.transpose()) // 对于旋转矩阵，逆等于转置
    }
}


class SE3(val rotation: SO3, val translation: Vec3d) {
    companion object {
        fun identity(): SE3 {
            return SE3(SO3.identity(), Vec3d(0.0))
        }

        fun create(pose: Pose): SE3 {
            return SE3(SO3.create(pose.quaternion), pose.translation)
        }
    }
    operator fun times(other: SE3): SE3 {
        val newRotation = this.rotation * other.rotation
        val newTranslation = this.rotation.matrix * other.translation + this.translation
        return SE3(newRotation, newTranslation)
    }
    operator fun times(point: Vec3d): Vec3d {
        val rotPoint = this.rotation.matrix * point
        return rotPoint + this.translation
    }
    fun inverse(): SE3 {
        val invRotation = this.rotation.inverse()
        val invTranslation = -(invRotation.matrix * this.translation)
        return SE3(invRotation, invTranslation)
    }
}