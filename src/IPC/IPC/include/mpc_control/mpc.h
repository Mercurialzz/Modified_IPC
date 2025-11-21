#ifndef MPC_H_
#define MPC_H_

#include <ros/ros.h>

#include <Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"

/* QP formulation:
    min 1/2* x^T H x + f^T x   subject to
    b <= Ax <= b (Ax = b),  d <= Ax <= f,  l <= x <= u
*/
struct mpc_osqp_t {
    Eigen::MatrixXd Ax;
    Eigen::MatrixXd Bx;
    Eigen::MatrixXd M;
    Eigen::MatrixXd C;
    Eigen::MatrixXd Q_bar;
    Eigen::MatrixXd R_bar, R_con_bar;

    Eigen::VectorXd u_low, u_upp, a_low, a_upp, v_low, v_upp;
    Eigen::VectorXd B_a, B_v, B_p;
    Eigen::MatrixXd A_a, A_v, A_p;
    Eigen::MatrixXd M_a, M_v, M_p;

    Eigen::MatrixXd T;
    Eigen::VectorXd D_T;

    Eigen::MatrixXd A_sys;
    Eigen::VectorXd A_sys_low, A_sys_upp;
    Eigen::MatrixXd A_sfc;
    Eigen::VectorXd A_sfc_low, A_sfc_upp;

    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    Eigen::MatrixXd A;
    Eigen::VectorXd Alow, Aupp;
    Eigen::SparseMatrix<double> H_sparse;
    Eigen::SparseMatrix<double> A_sparse;

    Eigen::VectorXd u_optimal;
};

class MPCPlannerClass {
public:
    MPCPlannerClass(){}
    MPCPlannerClass(ros::NodeHandle& nh) {
        init_param(nh);
        MPC_HORIZON = cfg_.MPC_HORIZON;
        MPC_STEP = cfg_.MPC_STEP;
        ctrl_delay_ = cfg_.ctrl_delay_;
        // ROS_INFO("v_max:%f", cfg_.v_max_.x());
        // std::cout << "v_max" << cfg_.v_max_.x() << " " << cfg_.v_max_.y() << " " << cfg_.v_max_.z() ;
        ProblemFormation();
        X_0_.resize(mpc_.M.cols(), 1);
        X_r_.resize(mpc_.M.rows(), 1);
        planes_.resize(MPC_HORIZON);
    }
    ~MPCPlannerClass() {}

    void GetOptimCmd(Eigen::Vector3d& u, int segment) {
        if (segment >= MPC_HORIZON) {
            segment = MPC_HORIZON - 1;
        }
        u.x() = mpc_.u_optimal(segment * 3 + 0, 0);
        u.y() = mpc_.u_optimal(segment * 3 + 1, 0);
        u.z() = mpc_.u_optimal(segment * 3 + 2, 0);
    }
    void SetGoal(Eigen::Vector3d pr, Eigen::Vector3d vr, Eigen::Vector3d ar, int step) {
        if (step > MPC_HORIZON || step < 0) {
            ROS_WARN("[MPC]: Check goal index! Error index: %d", step);
            return ;
        }
        Eigen::VectorXd x_0(X_0_.rows(), 1);
        x_0.block(0, 0, pr.rows(), 1) = pr;
        x_0.block(pr.rows(), 0, vr.rows(), 1) = vr;
        x_0.block(pr.rows()+vr.rows(), 0, ar.rows(), 1) = ar;
        X_r_.block(x_0.rows()*step, 0, x_0.rows(), 1) = x_0;
    }
    void SetStatus(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0) {
        StatusSaturation(v0, a0);
        X_0_.block(0, 0, p0.rows(), 1) = p0;
        X_0_.block(p0.rows(), 0, v0.rows(), 1) = v0;
        X_0_.block(p0.rows()+v0.rows(), 0, a0.rows(), 1) = a0;
        // for (int i = 0; i < ctrl_delay_; i++) {
        //     X_0_ = mpc_.Ax * X_0_ + mpc_.Bx * u_last_[i];
        // }
    }
    void StatusSaturation(Eigen::Vector3d& v0, Eigen::Vector3d& a0) {
        for (int i = 0; i < 3; i++) {
            if (v0(i, 0) > cfg_.v_max_(i, 0)) v0(i, 0) = cfg_.v_max_(i, 0);
            if (v0(i, 0) < cfg_.v_min_(i, 0)) v0(i, 0) = cfg_.v_min_(i, 0);
            if (a0(i, 0) > cfg_.a_max_(i, 0)) a0(i, 0) = cfg_.a_max_(i, 0);
            if (a0(i, 0) < cfg_.a_min_(i, 0)) a0(i, 0) = cfg_.a_min_(i, 0);
        }
    }
    void UpdateOutputHistory(Eigen::Vector3d u) {
        if (ctrl_delay_ == 0) return ;
        u_last_.erase(u_last_.begin());
        u_last_.push_back(u);
    }

    bool Run(void);
    void SystemModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double t);
    void SetFSC(Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, int step);
    bool IsInFSC(Eigen::Vector3d pos, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes){
        if (planes.rows() == 0) return false;
        for (int i = 0; i < planes.rows(); i++) {
            if (pos.x()*planes(i,0) + pos.y()*planes(i,1) + 
                pos.z()*planes(i,2) + planes(i,3) > 0) {
                // std::cout << "plane:" << i << " " << planes_.block<1, 4>(i, 0) << std::endl;
                return false;
            }
        }
        return true;
    }

    Eigen::VectorXd X_0_, X_r_;

    // mpc param
    int MPC_HORIZON;
    double MPC_STEP;
    int ctrl_delay_;
    std::vector<Eigen::Vector3d> u_last_;

private:
    void ProblemFormation(void);
    void MPCModel(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
                  Eigen::MatrixXd& M, Eigen::MatrixXd& C);
    void QuadraticTerm(mpc_osqp_t& mpc, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
                        const Eigen::MatrixXd& R_con, const Eigen::MatrixXd& F);
    void LinearTerm(mpc_osqp_t& mpc, const Eigen::VectorXd& x_0, const Eigen::VectorXd& x_r);
    void ALLConstraint(mpc_osqp_t& mpc);
    void UpdateBound(mpc_osqp_t& mpc, const Eigen::VectorXd& x_0);

    mpc_osqp_t mpc_;
    ros::Time print_time_;
    int fps_;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 4>> planes_;
    /*mpc param*/
    struct MPC_param
    {
        int MPC_HORIZON{15};
        double MPC_STEP{0.1};
        int ctrl_delay_{0};
        double R_p_{100.0}, R_v_{0.0}, R_a_{0.0};
        double R_u_{10.0}, R_u_con_{1.0};
        double R_pN_{0.0}, R_vN_{0.0}, R_aN_{0.0};
        Eigen::Matrix3d Drag_; //初始化
        Eigen::Vector3d v_min_, v_max_, a_min_, a_max_, u_min_, u_max_;
    } cfg_;

    void init_param(const ros::NodeHandle &nh)
    {
        cfg_.Drag_.setZero();
        read_essential_param(nh, "mpc/horizon", cfg_.MPC_HORIZON);
        read_essential_param(nh, "mpc/step",    cfg_.MPC_STEP);
        // read_essential_param(nh, "mpc/ctrl_delay", ctrl_delay_);
        u_last_.resize(cfg_.ctrl_delay_);
        for (int i = 0; i < u_last_.size(); i++) {
            u_last_[i] = Eigen::Vector3d::Zero();
        }

        read_essential_param(nh, "mpc/R_p",  cfg_.R_p_);
        read_essential_param(nh, "mpc/R_v",  cfg_.R_v_);
        read_essential_param(nh, "mpc/R_a",  cfg_.R_a_);
        read_essential_param(nh, "mpc/R_u",  cfg_.R_u_);
        read_essential_param(nh, "mpc/R_u_con",  cfg_.R_u_con_);
        read_essential_param(nh, "mpc/R_pN", cfg_.R_pN_);
        read_essential_param(nh, "mpc/R_vN", cfg_.R_vN_);
        read_essential_param(nh, "mpc/R_aN", cfg_.R_aN_);

        read_essential_param(nh, "mpc/D_x", cfg_.Drag_(0, 0));
        read_essential_param(nh, "mpc/D_y", cfg_.Drag_(1, 1));
        read_essential_param(nh, "mpc/D_z", cfg_.Drag_(2, 2));

        read_essential_param(nh, "mpc/vx_min", cfg_.v_min_.x());
        read_essential_param(nh, "mpc/vy_min", cfg_.v_min_.y());
        read_essential_param(nh, "mpc/vz_min", cfg_.v_min_.z());
        read_essential_param(nh, "mpc/vx_max", cfg_.v_max_.x());
        read_essential_param(nh, "mpc/vy_max", cfg_.v_max_.y());
        read_essential_param(nh, "mpc/vz_max", cfg_.v_max_.z());

        read_essential_param(nh, "mpc/ax_min", cfg_.a_min_.x());
        read_essential_param(nh, "mpc/ay_min", cfg_.a_min_.y());
        read_essential_param(nh, "mpc/az_min", cfg_.a_min_.z());
        read_essential_param(nh, "mpc/ax_max", cfg_.a_max_.x());
        read_essential_param(nh, "mpc/ay_max", cfg_.a_max_.y());
        read_essential_param(nh, "mpc/az_max", cfg_.a_max_.z());
        read_essential_param(nh, "mpc/ux_min", cfg_.u_min_.x());
        read_essential_param(nh, "mpc/uy_min", cfg_.u_min_.y());
        read_essential_param(nh, "mpc/uz_min", cfg_.u_min_.z());
        read_essential_param(nh, "mpc/ux_max", cfg_.u_max_.x());
        read_essential_param(nh, "mpc/uy_max", cfg_.u_max_.y());
        read_essential_param(nh, "mpc/uz_max", cfg_.u_max_.z());
        
    }
    template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// 打印参数值
            ROS_INFO_STREAM("Read param: " << name << " successfully, value: " << val);
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};

};

#endif
