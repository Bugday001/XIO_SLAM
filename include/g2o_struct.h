#ifndef G2O_STRUCT_
#define G2O_STRUCT_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>
#include "g2o/core/robust_kernel_impl.h"

#include "imuPreintFun.h"
#include "mathFun.h"

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /**
         * 顶点是可以设置成固定的。当不需要变动某个顶点时，使用setFixed函数来固定。
         * 通常，一个优化问题中，至少需要固定一个顶点，否则所有的顶点都在浮动，优化效果也不会好。
         */
        VertexPose(bool fixed = false)
        {
            setToOriginImpl();
            setFixed(fixed);
        }

        VertexPose(const Sophus::SE3d e, bool fixed = false)
        {
            _estimate = e;
            setFixed(fixed);
        }

        /**
         * 用于重置顶点的数据。顶点包含的数据变量是_estimate。
         * 该变量的类型即是g2o::BaseVertex<1, double>中设置的double。
         * 该函数正是用于重置_estimate和使顶点恢复默认状态。
         */
        virtual void setToOriginImpl() override
        {
            _estimate = Sophus::SE3d();
            ;
        }

        /**
         * 用于叠加优化量的步长。注意有时候这样的叠加操作并不是线性的。
         */
        virtual void oplusImpl(const double *update) override
        {
            // Eigen::Matrix<double, 6, 1> delta_r;
            // delta_r << update[0], update[1], update[2], update[3], update[4], update[5];
            // std::cout<<"detla_r:";
            // for(int i=0; i<6; i++)
            //     std::cout<<update[i]<<", ";
            // std::cout<<std::endl;
            // _estimate = _estimate * Sophus::SE3d::exp(delta_r);
            //也可以分开计算
            _estimate.so3() = _estimate.so3() * Sophus::SO3d::exp(Eigen::Map<const Eigen::Vector3d>(&update[0]));  // 旋转部分
            _estimate.translation() += Eigen::Map<const Eigen::Vector3d>(&update[3]);                     // 平移部分
        }

        // 存盘和读盘：留空
        virtual bool read(std::istream &in) { return true; }
        virtual bool write(std::ostream &out) const { return true; }
};

/**
 * 速度Vertex
 */
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    virtual void setToOriginImpl() { _estimate.setZero(); }

    virtual void oplusImpl(const double* update_) { _estimate += Eigen::Map<const Eigen::Vector3d>(update_); }
};

/**
 * 陀螺零偏顶点，直接使用速度的class
 */
class VertexGyroBias : public VertexVelocity {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias() {}
};

/**
 * 加速度计
 */
class VertexAccBias : public VertexVelocity {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAccBias() {}
};

/********************************************以下为边*********************************************************/
/**
 * 多元边
*/
class EdgeInertial : public g2o::BaseMultiEdge<9, Eigen::Vector3d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * 构造函数中需要指定预积分类对象
     * @param preinteg  预积分对象指针
     * @param weight    权重
     */
    EdgeInertial(std::shared_ptr<xio::IMUPreintegration> preinteg, double weight = 1.0)    
    : preint_(preinteg), dt_(preinteg->dt_) {
        resize(6);  // 6个关联顶点
        grav_ = preinteg->gw_;
        setInformation(preinteg->cov_.inverse() * weight);
    }

    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

    void computeError() {
        //static_cast和dynamic_cast，https://blog.csdn.net/andybegin/article/details/125050822
        auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
        auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
        auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
        auto* ba1 = dynamic_cast<const VertexAccBias*>(_vertices[3]);
        auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
        auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

        Eigen::Vector3d bg = bg1->estimate();
        Eigen::Vector3d ba = ba1->estimate();

        const Sophus::SO3d dR = preint_->GetDeltaRotation(bg);
        const Eigen::Vector3d dv = preint_->GetDeltaVelocity(bg, ba);
        const Eigen::Vector3d dp = preint_->GetDeltaPosition(bg, ba);

        /// 预积分误差项（4.41）
        const Eigen::Vector3d er = (dR.inverse() * p1->estimate().so3().inverse() * p2->estimate().so3()).log();
        Eigen::Matrix3d RiT = p1->estimate().so3().inverse().matrix();
        const Eigen::Vector3d ev = RiT * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv;
        const Eigen::Vector3d ep = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ -
                                grav_ * dt_ * dt_ / 2) - dp;
        // std::cout<<"computeError"<<er<<ev<<ep<<std::endl;
        _error << er, ev, ep;
    }
    void linearizeOplus() {
        auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
        auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
        auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
        auto* ba1 = dynamic_cast<const VertexAccBias*>(_vertices[3]);
        auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
        auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

        Eigen::Vector3d bg = bg1->estimate();
        Eigen::Vector3d ba = ba1->estimate();
        Eigen::Vector3d dbg = bg - preint_->bg_;

        // 一些中间符号
        const Sophus::SO3d R1 = p1->estimate().so3();
        const Sophus::SO3d R1T = R1.inverse();
        const Sophus::SO3d R2 = p2->estimate().so3();

        auto dR_dbg = preint_->dR_dbg_;
        auto dv_dbg = preint_->dV_dbg_;
        auto dp_dbg = preint_->dP_dbg_;
        auto dv_dba = preint_->dV_dba_;
        auto dp_dba = preint_->dP_dba_;

        // 估计值
        Eigen::Vector3d vi = v1->estimate();
        Eigen::Vector3d vj = v2->estimate();
        Eigen::Vector3d pi = p1->estimate().translation();
        Eigen::Vector3d pj = p2->estimate().translation();

        const Sophus::SO3d dR = preint_->GetDeltaRotation(bg);
        const Sophus::SO3d eR = Sophus::SO3d(dR).inverse() * R1T * R2;
        const Eigen::Vector3d er = eR.log();
        const Eigen::Matrix3d invJr = xio::jr_inv(eR.log());

        /// 雅可比矩阵
        /// 注意有3个index, 顶点的，自己误差的，顶点内部变量的
        /// 变量顺序：pose1(R1,p1), v1, bg1, ba1, pose2(R2,p2), v2
        /// 残差顺序：eR, ev, ep，残差顺序为行，变量顺序为列

        //       | R1 | p1 | v1 | bg1 | ba1 | R2 | p2 | v2 |
        //  vert | 0       | 1  | 2   | 3   | 4       | 5  |
        //  col  | 0    3  | 0  | 0   | 0   | 0    3  | 0  |
        //    row
        //  eR 0 |
        //  ev 3 |
        //  ep 6 |

        /// 残差对R1, 9x3
        _jacobianOplus[0].setZero();
        // dR/dR1, 4.42
        _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * (R2.inverse() * R1).matrix();
        // dv/dR1, 4.47
        _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(R1T * (vj - vi - grav_ * dt_));
        // dp/dR1, 4.48d
        _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat(R1T * (pj - pi - v1->estimate() * dt_ - 0.5 * grav_ * dt_ * dt_));

        /// 残差对p1, 9x3
        // dp/dp1, 4.48a
        _jacobianOplus[0].block<3, 3>(6, 3) = -R1T.matrix();

        /// 残差对v1, 9x3
        _jacobianOplus[1].setZero();
        // dv/dv1, 4.46a
        _jacobianOplus[1].block<3, 3>(3, 0) = -R1T.matrix();
        // dp/dv1, 4.48c
        _jacobianOplus[1].block<3, 3>(6, 0) = -R1T.matrix() * dt_;

        /// 残差对bg1
        _jacobianOplus[2].setZero();
        // dR/dbg1, 4.45
        _jacobianOplus[2].block<3, 3>(0, 0) = -invJr * eR.inverse().matrix() * xio::jr((dR_dbg * dbg).eval()) * dR_dbg;
        // dv/dbg1
        _jacobianOplus[2].block<3, 3>(3, 0) = -dv_dbg;
        // dp/dbg1
        _jacobianOplus[2].block<3, 3>(6, 0) = -dp_dbg;

        /// 残差对ba1
        _jacobianOplus[3].setZero();
        // dv/dba1
        _jacobianOplus[3].block<3, 3>(3, 0) = -dv_dba;
        // dp/dba1
        _jacobianOplus[3].block<3, 3>(6, 0) = -dp_dba;

        /// 残差对pose2
        _jacobianOplus[4].setZero();
        // dr/dr2, 4.43
        _jacobianOplus[4].block<3, 3>(0, 0) = invJr;
        // dp/dp2, 4.48b
        _jacobianOplus[4].block<3, 3>(6, 3) = R1T.matrix();

        /// 残差对v2
        _jacobianOplus[5].setZero();
        // dv/dv2, 4,46b
        _jacobianOplus[5].block<3, 3>(3, 0) = R1T.matrix();  // OK
        // std::cout<<"_jacobianOplus:"<<std::endl;
        // for(int i=0; i<6; i++) {
        //     std::cout<<_jacobianOplus[i]<<std::endl;
        // }
    }

    Eigen::Matrix<double, 24, 24> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[0];
        J.block<9, 3>(0, 6) = _jacobianOplus[1];
        J.block<9, 3>(0, 9) = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

   private:
    const double dt_;
    std::shared_ptr<xio::IMUPreintegration> preint_ = nullptr;
    Eigen::Vector3d grav_;
};

/**
 * 陀螺随机游走
 */
class EdgeGyroRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexGyroBias, VertexGyroBias> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyroRW() {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError() {
        const auto* VG1 = dynamic_cast<const VertexGyroBias*>(_vertices[0]);
        const auto* VG2 = dynamic_cast<const VertexGyroBias*>(_vertices[1]);
        _error = VG2->estimate() - VG1->estimate();
    }

    virtual void linearizeOplus() {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

/**
 * 加计随机游走
 */
class EdgeAccRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias, VertexAccBias> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAccRW() {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError() {
        const auto* VA1 = dynamic_cast<const VertexAccBias*>(_vertices[0]);
        const auto* VA2 = dynamic_cast<const VertexAccBias*>(_vertices[1]);
        _error = VA2->estimate() - VA1->estimate();
    }

    virtual void linearizeOplus() {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }
};

/**
 * 对上一帧IMU pvq bias的先验
 * info 由外部指定，通过时间窗口边缘化给出
 *
 * 顶点顺序：pose, v, bg, ba
 * 残差顺序：R, p, v, bg, ba，15维
 */
class EdgePriorPoseNavState : public g2o::BaseMultiEdge<15, Eigen::Matrix<double, 15, 1>> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorPoseNavState(const xio::NavState& state, const Eigen::Matrix<double, 15, 15>& info) {
        resize(4);  
        state_ = state;
        setInformation(info);
    }

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError() {
        auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
        auto* vv = dynamic_cast<const VertexVelocity*>(_vertices[1]);
        auto* vg = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
        auto* va = dynamic_cast<const VertexAccBias*>(_vertices[3]);

        const Eigen::Vector3d er = Sophus::SO3d(state_.qwb.matrix().transpose() * vp->estimate().so3().matrix()).log();
        const Eigen::Vector3d ep = vp->estimate().translation() - state_.Pwb;
        const Eigen::Vector3d ev = vv->estimate() - state_.Vw;
        const Eigen::Vector3d ebg = vg->estimate() - state_.bg_;
        const Eigen::Vector3d eba = va->estimate() - state_.ba_;
        // std::cout<<"computeError2"<<er<<ep<<ev<<ebg<<eba<<std::endl;
        _error << er, ep, ev, ebg, eba;
    }
    void linearizeOplus() {
        const auto* vp = dynamic_cast<const VertexPose*>(_vertices[0]);
        const Eigen::Vector3d er = Sophus::SO3d(state_.qwb.matrix().transpose() * vp->estimate().so3().matrix()).log();

        /// 注意有3个index, 顶点的，自己误差的，顶点内部变量的
        _jacobianOplus[0].setZero();
        _jacobianOplus[0].block<3, 3>(0, 0) = xio::jr_inv(er);    // dr/dr
        _jacobianOplus[0].block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();  // dp/dp
        _jacobianOplus[1].setZero();
        _jacobianOplus[1].block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();  // dv/dv
        _jacobianOplus[2].setZero();
        _jacobianOplus[2].block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();  // dbg/dbg
        _jacobianOplus[3].setZero();
        _jacobianOplus[3].block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();  // dba/dba
            // std::cout<<"_jacobianOplus2:"<<std::endl;
        // for(int i=0; i<4; i++) {
        //     std::cout<<_jacobianOplus[i]<<std::endl;
        // }
    }

    Eigen::Matrix<double, 15, 15> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 15, 15> J;
        J.block<15, 6>(0, 0) = _jacobianOplus[0];
        J.block<15, 3>(0, 6) = _jacobianOplus[1];
        J.block<15, 3>(0, 9) = _jacobianOplus[2];
        J.block<15, 3>(0, 12) = _jacobianOplus[3];
        return J.transpose() * information() * J;
    }

    xio::NavState state_;
};

/**
 * 6 自由度的GNSS
 * 误差的角度在前，平移在后
 */
class EdgeGNSS : public g2o::BaseUnaryEdge<6, Sophus::SE3d, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeGNSS() = default;
    EdgeGNSS(VertexPose* v, const Sophus::SE3d& obs) {
        setVertex(0, v);
        setMeasurement(obs);
    }

    void computeError() override {
        VertexPose* v = (VertexPose*)_vertices[0];
        _error.head<3>() = (_measurement.so3().inverse() * v->estimate().so3()).log();
        _error.tail<3>() = v->estimate().translation() - _measurement.translation();
    };

    void linearizeOplus() override {
        VertexPose* v = (VertexPose*)_vertices[0];
        // jacobian 6x6
        _jacobianOplusXi.setZero();
        _jacobianOplusXi.block<3, 3>(0, 0) = xio::jr_inv((_measurement.so3().inverse() * v->estimate().so3()).log());  // dR/dR
        _jacobianOplusXi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();                                              // dp/dp
    }

    Matrix<double, 6, 6> GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

    virtual bool read(std::istream& in) { return true; }
    virtual bool write(std::ostream& out) const { return true; }

   private:
};

#endif