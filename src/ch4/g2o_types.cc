//
// Created by xiang on 23-1-19.
//

#include "ch4/g2o_types.h"
#include "common/g2o_types.h"

namespace sad {

EdgeInertial::EdgeInertial(std::shared_ptr<IMUPreintegration> preinteg, const Vec3d& gravity, double weight)
    : preint_(preinteg), dt_(preinteg->dt_) {
    resize(6);  // 6个关联顶点
    grav_ = gravity;
    setInformation(preinteg->cov_.inverse() * weight);
}

void EdgeInertial::computeError() {
    auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* ba1 = dynamic_cast<const VertexAccBias*>(_vertices[3]);
    auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
    auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

    Vec3d bg = bg1->estimate();
    Vec3d ba = ba1->estimate();

    const SO3 dR = preint_->GetDeltaRotation(bg);
    const Vec3d dv = preint_->GetDeltaVelocity(bg, ba);
    const Vec3d dp = preint_->GetDeltaPosition(bg, ba);

    /// 预积分误差项（4.41）
    const Vec3d er = (dR.inverse() * p1->estimate().so3().inverse() * p2->estimate().so3()).log();
    Mat3d RiT = p1->estimate().so3().inverse().matrix();
    const Vec3d ev = RiT * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv;
    const Vec3d ep = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ -
                            grav_ * dt_ * dt_ / 2) -
                     dp;
    _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus() {
    auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* ba1 = dynamic_cast<const VertexAccBias*>(_vertices[3]);
    auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
    auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

    Vec3d bg = bg1->estimate();
    Vec3d ba = ba1->estimate();
    Vec3d dbg = bg - preint_->bg_;

    // 一些中间符号
    const SO3 R1 = p1->estimate().so3();
    const SO3 R1T = R1.inverse();
    const SO3 R2 = p2->estimate().so3();

    auto dR_dbg = preint_->dR_dbg_;
    auto dv_dbg = preint_->dV_dbg_;
    auto dp_dbg = preint_->dP_dbg_;
    auto dv_dba = preint_->dV_dba_;
    auto dp_dba = preint_->dP_dba_;

    // 估计值
    Vec3d vi = v1->estimate();
    Vec3d vj = v2->estimate();
    Vec3d pi = p1->estimate().translation();
    Vec3d pj = p2->estimate().translation();

    const SO3 dR = preint_->GetDeltaRotation(bg);
    const SO3 eR = SO3(dR).inverse() * R1T * R2;
    const Vec3d er = eR.log();
    const Mat3d invJr = SO3::jr_inv(eR);

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
    _jacobianOplus[0].block<3, 3>(3, 0) = SO3::hat(R1T * (vj - vi - grav_ * dt_));
    // dp/dR1, 4.48d
    _jacobianOplus[0].block<3, 3>(6, 0) = SO3::hat(R1T * (pj - pi - v1->estimate() * dt_ - 0.5 * grav_ * dt_ * dt_));

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
    _jacobianOplus[2].block<3, 3>(0, 0) = -invJr * eR.inverse().matrix() * SO3::jr((dR_dbg * dbg).eval()) * dR_dbg;
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
}

}  // namespace sad