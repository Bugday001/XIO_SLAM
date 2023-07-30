#ifndef _MATH_FUN_
#define _MATH_FUN_

#include <Eigen/Dense>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

using namespace Eigen;

namespace xio{
    inline Matrix3d jl(const Vector3d& Omega) {
        double theta = Omega.norm();
        if (theta < 1e-6) {
            return Matrix3d::Identity();
        }

        Vector3d a = Omega;
        a.normalize();
        double sin_theta = std::sin(theta);
        double cos_theta = std::cos(theta);
        return (sin_theta / theta) * Matrix3d ::Identity() + (1 - sin_theta / theta) * a * a.transpose() +
            (1 - cos_theta) / theta * Sophus::SO3d::hat(a);
    }
    inline Matrix3d jl_inv(const Vector3d& Omega) {
        double theta = Omega.norm();
        if (theta < 1e-6) {
            return Matrix3d::Identity();
        }

        Vector3d a = Omega;
        a.normalize();

        double cot_half_theta = std::cos(0.5 * theta) / std::sin(0.5 * theta);
        return 0.5 * theta * cot_half_theta * Matrix3d ::Identity() +
               (1 - 0.5 * theta * cot_half_theta) * a * a.transpose() - 0.5 * theta * Sophus::SO3d::hat(a);
    }
    inline Matrix3d jr_inv(const Vector3d& Omega) { return jl_inv(-Omega); }
    inline Matrix3d jr(const Vector3d& Omega) {return jl(-Omega);}

    /**
     * 边缘化
     * @param H
     * @param start
     * @param end
     * @return
     */
    inline Eigen::MatrixXd Marginalize(const Eigen::MatrixXd& H, const int& start, const int& end) {
        // Goal
        // a  | ab | ac       a*  | 0 | ac*
        // ba | b  | bc  -->  0   | 0 | 0
        // ca | cb | c        ca* | 0 | c*

        // Size of block before block to marginalize
        const int a = start;
        // Size of block to marginalize
        const int b = end - start + 1;
        // Size of block after block to marginalize
        const int c = H.cols() - (end + 1);

        // Reorder as follows:
        // a  | ab | ac       a  | ac | ab
        // ba | b  | bc  -->  ca | c  | cb
        // ca | cb | c        ba | bc | b

        Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
        if (a > 0) {
            Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
            Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
            Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
        }
        if (a > 0 && c > 0) {
            Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
            Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
        }
        if (c > 0) {
            Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
            Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
            Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
        }
        Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

        // Perform marginalization (Schur complement)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
        for (int i = 0; i < b; ++i) {
            if (singularValues_inv(i) > 1e-6) singularValues_inv(i) = 1.0 / singularValues_inv(i);
            else
                singularValues_inv(i) = 0;
        }
        Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
        Hn.block(0, 0, a + c, a + c) =
            Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
        Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
        Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
        Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

        // Inverse reorder
        // a*  | ac* | 0       a*  | 0 | ac*
        // ca* | c*  | 0  -->  0   | 0 | 0
        // 0   | 0   | 0       ca* | 0 | c*
        Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
        if (a > 0) {
            res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
            res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
            res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
        }
        if (a > 0 && c > 0) {
            res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
            res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
        }
        if (c > 0) {
            res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
            res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
            res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
        }

        res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

        return res;
    }
}

#endif