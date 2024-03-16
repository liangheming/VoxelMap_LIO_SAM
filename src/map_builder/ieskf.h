#pragma once
#include "commons.h"

namespace lio
{
    const double GRAVITY = 9.81;

    using Vector21d = Eigen::Matrix<double, 21, 1>;
    using Vector12d = Eigen::Matrix<double, 12, 1>;
    using Matrix21d = Eigen::Matrix<double, 21, 21>;
    using Matrix12d = Eigen::Matrix<double, 12, 12>;
    using Matrix21x12d = Eigen::Matrix<double, 21, 12>;
    using Matrix23x12d = Eigen::Matrix<double, 23, 12>;

    using Matrix23d = Eigen::Matrix<double, 23, 23>;
    using Vector23d = Eigen::Matrix<double, 23, 1>;
    using Vector24d = Eigen::Matrix<double, 24, 1>;
    using Matrix3x2d = Eigen::Matrix<double, 3, 2>;
    using Matrix2x3d = Eigen::Matrix<double, 2, 3>;

    struct SharedState
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Matrix12d H;
        Vector12d b;
        size_t iter_num = 0;
    };
    struct State
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d rot_ext = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos_ext = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d bg = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba = Eigen::Vector3d::Zero();
        Eigen::Vector3d g = Eigen::Vector3d(0.0, 0.0, -GRAVITY);
        void initG(const Eigen::Vector3d &gravity_dir)
        {
            g = gravity_dir.normalized() * GRAVITY;
        }
        void operator+=(const Vector23d &delta);

        void operator+=(const Vector24d &delta);

        Vector23d operator-(const State &other);

        Matrix3x2d getBx() const;

        Matrix3x2d getMx() const;

        Matrix3x2d getMx(const Eigen::Vector2d &res) const;

        Matrix2x3d getNx() const;
    };

    struct Input
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        Input() = default;
        Input(Eigen::Vector3d &a, Eigen::Vector3d &g) : acc(a), gyro(g) {}
    };

    using measure_func = std::function<void(State &, SharedState &)>;

    class IESKF
    {
    public:
        static int I_P, I_R, I_ER, I_EP, I_V, I_BG, I_BA, I_G;
        IESKF();

        IESKF(size_t max_iter) : max_iter_(max_iter) {}

        void setMaxIter(int max_iter)
        {
            assert(max_iter > 0);
            max_iter_ = max_iter;
        }

        State &x() { return x_; }

        Matrix23d &P() { return P_; }

        void set_share_function(measure_func func) { func_ = func; }

        void predict(const Input &inp, double dt, const Matrix12d &Q);

        void update();

    private:
        size_t max_iter_ = 5;
        double eps_ = 0.001;
        State x_;
        Matrix23d P_;
        measure_func func_;
        Matrix23d H_;
        Vector23d b_;
        Matrix23d F_;
        Matrix23x12d G_;
    };
}
