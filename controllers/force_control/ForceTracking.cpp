#include "ForceTracking.hpp"

ForceTracker::ForceTracker(Eigen::Matrix2d M, Eigen::Matrix2d K, Eigen::Matrix2d D, Eigen::Vector2d a_kp, Eigen::Vector2d a_ki,
                           Eigen::Vector2d a_kd)
{
    M_d = M;
    K_0 = K;
    D_d = D;
    adaptive_kp = a_kp;
    adaptive_ki = a_ki;
    adaptive_kd = a_kd;

    force_tracker_x.kp = a_kp[0];
    force_tracker_x.ki = a_ki[0];
    force_tracker_x.kd = a_kd[0];
    force_tracker_x.err_sum_lim = 10000;
    force_tracker_x.dT = T_;

    force_tracker_y.kp = a_kp[1];
    force_tracker_y.ki = a_ki[1];
    force_tracker_y.kd = a_kd[1];
    force_tracker_y.err_sum_lim = 12;
    force_tracker_y.dT = T_;

    trq_lpf_r.init(20, T_);
    trq_lpf_l.init(20, T_);
    vel_lpf_r.init(20, T_);
    vel_lpf_l.init(20, T_);
    dtheta_lpf.init(20, T_);
    dbeta_lpf.init(20, T_);
    ddtheta_lpf.init(10, T_);
    ddbeta_lpf.init(10, T_);
}

void ForceTracker::initialize(const Eigen::Vector2d& init_tb)
{
    // initialize state queue
    Eigen::Vector2d z(0, 0);
    Eigen::Vector2d init_xy = fk(init_tb);

    X_d_q.push_front(init_xy);
    X_d_q.push_front(init_xy);
    X_d_q.push_front(init_xy);

    F_d_q.push_front(z);
    F_d_q.push_front(z);
    F_d_q.push_front(z);

    X_c_q.push_front(init_xy);
    X_c_q.push_front(init_xy);

    TB_fb_q.push_front(init_tb);
    TB_fb_q.push_front(init_tb);
    TB_fb_q.push_front(init_tb);
    TB_fb_q.push_front(init_tb);

    T_fb_q.push_front(z);
    T_fb_q.push_front(z);
    T_fb_q.push_front(z);

    F_fb_q.push_front(z);
    F_fb_q.push_front(z);
    F_fb_q.push_front(z);

    adaptive_pid_out.push_front(z);
    adaptive_pid_out.push_front(z);
    adaptive_pid_out.push_front(z);
    adaptive_pid_err.push_front(z);
    adaptive_pid_err.push_front(z);
    adaptive_pid_err.push_front(z);
}

Eigen::Vector2d ForceTracker::track(const Eigen::Vector2d& X_d, const Eigen::Vector2d& F_d, const Eigen::Matrix2d& K_adapt)
{
    Eigen::Matrix2d K_d = K_0 + K_adapt;
    K_ft = K_d;
    // Eigen::Matrix2d K_d = K_0;
    Eigen::Vector2d Xc_k = PositionBasedImpFilter(M_d, K_d, D_d, X_d_q, F_d_q, X_c_q, TB_fb_q, T_fb_q, F_fb_q);
    update_delay_state<Eigen::Vector2d>(X_c_q, Xc_k);

    Eigen::Vector2d tb_k = ik(Xc_k);
    Eigen::Vector2d phi_k = tb2phi(tb_k);

    return phi_k;
}

Eigen::Vector2d ForceTracker::controlLoop(const Eigen::Vector2d& X_d, const Eigen::Vector2d& F_d, const Eigen::Vector2d& tb_fb,
                                          const Eigen::Vector2d& trq_fb_filt, const Eigen::Vector2d& phi_vel_fb_filt)
{
    
    update_delay_state<Eigen::Vector2d>(TB_fb_q, tb_fb);
    update_delay_state<Eigen::Vector2d>(T_fb_q, trq_fb_filt);
    update_delay_state<Eigen::Vector2d>(X_d_q, X_d);
    update_delay_state<Eigen::Vector2d>(F_d_q, F_d);


    // Filter State for Force Estimation (inverse Dynamic)
    Eigen::Vector2d dtb = TB_fb_q[0] - TB_fb_q[1];
    Eigen::Vector2d dtb_1 = TB_fb_q[1] - TB_fb_q[2];
    Eigen::Vector2d ddtb = dtb - dtb_1;

    /*
    double dtheta_filt = dtheta_lpf.update(dtb[0]);
    double dbeta_filt = dbeta_lpf.update(dtb[1]);
    double ddtheta_filt = ddtheta_lpf.update(ddtb[0]);
    double ddbeta_filt = ddbeta_lpf.update(ddtb[1]);
    Eigen::Vector2d dTB_filt(dtheta_filt, dbeta_filt);
    Eigen::Vector2d ddTB_filt(ddtheta_filt, ddbeta_filt);
    */

    // Eigen::Vector2d tau_friction = jointFriction(phi_vel_fb_filt);
    Eigen::Vector2d tau_friction(0.0, 0.0);
    
    // Force Estimation only with Virtual Work Method
    // Eigen::Vector2d F_est_l2g =
    //     forceEstimation(trq_fb_filt, TB_fb_q, tau_friction);  // force leg to ground

    // Force Estimation with the inertia term and filtered state input

    Eigen::Vector2d F_est_l2g =
        forceEstimation(T_fb_q[0], TB_fb_q[0], dtb, ddtb, tau_friction);  // force leg to ground

    Eigen::Vector2d F_est_g2l = -1 * F_est_l2g;  // GRF ground to leg
    Eigen::Vector2d F_err_g2l = F_d - F_est_g2l;

    update_delay_state<Eigen::Vector2d>(F_fb_q, F_est_l2g);

    printf("Force Estimated = [%lf, %lf]\n", F_est_g2l[0], F_est_g2l[1]);

    // adaptive stiffness
    Eigen::Matrix2d K_adpt;

    double ka_x = force_tracker_x.tracking(F_d[0], F_est_g2l[0]);
    double ka_y = force_tracker_y.tracking(F_d[1], F_est_g2l[1]);
    if (F_d[0] < 0)
    {
        ka_x *= -1;
    }
    K_adpt << ka_x, 0, 0, ka_y;


    /*
    printf("T_fb_q[0] = [%lf, %lf]\n",T_fb_q[0][0], T_fb_q[0][1]);
    printf("TB_fb_q[0] = [%lf, %lf]\n",TB_fb_q[0][0], TB_fb_q[0][1]);
    printf("dtb = [%lf, %lf]\n",dtb[0], dtb[1]);
    printf("ddtb = [%lf, %lf]\n",ddtb[0], ddtb[1]);
    printf("tau_friction = [%lf, %lf]\n",tau_friction[0], tau_friction[1]);
    printf("F_est_l2g = [%lf, %lf]\n",F_est_l2g[0], F_est_l2g[1]);
    printf("X_d = [%lf, %lf]\n",X_d[0], X_d[1]);
    printf("F_d = [%lf, %lf]\n",F_d[0], F_d[1]);
    printf("K_adpt = [%lf, %lf]\n",ka_x, ka_y);
    */

    // track impedance trajectory
    Eigen::Vector2d phi = track(X_d, F_d, K_adpt);

    /* term << F_d[0] << "," << F_est_g2l[0] << ",";
    term << force_tracker_x.kp << "," << force_tracker_x.ki << "," << force_tracker_x.kd << ",";
    term << force_tracker_x.err << "," << force_tracker_x.err_sum << "," << force_tracker_x.err_dev
         << "," << ka_x << ",";

    term << F_d[1] << "," << F_est_g2l[1] << ",";
    term << force_tracker_y.kp << "," << force_tracker_y.ki << "," << force_tracker_y.kd << ",";
    term << force_tracker_y.err << "," << force_tracker_y.err_sum << "," << force_tracker_y.err_dev
         << "," << ka_y << "\n"; */

    return phi;
}

Eigen::Vector2d ForceTracker::jointFriction(const Eigen::Vector2d& v_phi)
{
    double tf_R = stribeckFrictionModel(0, v_phi[0]);
    double tf_L = stribeckFrictionModel(1, v_phi[1]);
    Eigen::Vector2d t_friction(tf_R, tf_L);
    return t_friction;
}

double ForceTracker::stribeckFrictionModel(int idx, double v)
{
    double v_st = breakaway_vel_[idx] * sqrt(2);
    double v_coul = breakaway_vel_[idx] / 10;
    double e = std::exp(1);
    double F = sqrt(2 * e) * (breakaway_Ft_[idx] - coulumb_Ft_[idx]) * std::exp(-pow((v / v_st), 2)) * v / v_st +
               coulumb_Ft_[idx] * tanh(v / v_coul) + viscous_cff_[idx] * v;
    return F;
}

template <typename T>
void ForceTracker::update_delay_state(std::deque<T>& state, T x)
{
    state.push_front(x);
    state.pop_back();
}


Eigen::Vector2d PositionBasedImpFilter(const Eigen::Matrix2d& M, const Eigen::Matrix2d& K, const Eigen::Matrix2d& D,
                                       const std::deque<Eigen::Vector2d>& Xref, const std::deque<Eigen::Vector2d>& Fref,
                                       const std::deque<Eigen::Vector2d>& Xc, const std::deque<Eigen::Vector2d>& TB_fb,
                                       const std::deque<Eigen::Vector2d>& T_fb, const std::deque<Eigen::Vector2d>& F_fb)
{
    /* Xref = [x_k, y_k;
              x_k_1, y_k_1;
              x_k_2, y_k_2;] */
    /* Xc = [xc_k_1, yc_k_1;
            yc_k_2, yc_k_2] */
    /* Fref = [Fx(k), Fy(k);
               Fx(k-1), Fy(k-1);
               Fx(k-2), Fy(k-2)] */
    /* TB_fb = [theta(k), beta(k);
                theta(k-1), beta(k-1);
                theta(k-2), beta(k-2)
                theta(k-3), beta(k-3)] */
    /* T_fb = [T_R(k), T_L(k);
               T_R(k-1), T_L(k-1);
               T_R(k-2), T_L(k-2)] */
    /* Fext: Force exert to ground (Obtain by Virtual work method)*/

    Eigen::Vector2d X_k = fk(TB_fb[0]);
    Eigen::Vector2d X_k_1 = fk(TB_fb[1]);
    Eigen::Vector2d X_k_2 = fk(TB_fb[2]);

    Eigen::Vector2d d_F_k = F_fb[0];
    Eigen::Vector2d d_F_k_1 = F_fb[1];
    Eigen::Vector2d d_F_k_2 = F_fb[2];

    Eigen::Vector2d E_k_1 = Xref[1] - Xc[0];
    Eigen::Vector2d E_k_2 = Xref[2] - Xc[1];

    Eigen::Matrix<double, 2, 2> w1 = K * pow(T_, 2) + 2 * D * T_ + 4 * M;
    Eigen::Matrix<double, 2, 2> w2 = 2 * K * pow(T_, 2) - 8 * M;
    Eigen::Matrix<double, 2, 2> w3 = K * pow(T_, 2) - 2 * D * T_ + 4 * M;

    Eigen::Vector2d E_k;
    E_k = w1.inverse() * (pow(T_, 2) * (d_F_k + 2 * d_F_k_1 + d_F_k_2) - w2 * E_k_1 - w3 * E_k_2);

    // term << d_F_k[0] << "," << d_F_k[1] << "," << d_F_k_1[0] << "," << d_F_k_1[1] << "," <<
    // d_F_k_2[0] << ","
    //      << d_F_k_2[1] << "," << E_k[0] << "," << E_k[1] << "," << E_k_1[0] << "," << E_k_1[1] <<
    //      "," << E_k_2[0] << ","
    //      << E_k_2[1] << "," << Xref[0][0] << "," << Xref[0][1] << "," << w1.inverse()(0, 0) <<
    //      "," << w1.inverse()(1, 1)
    //      << "\n";

    Eigen::Vector2d Xc_k = Xref[0] - E_k;

    return Xc_k;
}

Eigen::Vector2d forceEstimation(const Eigen::Vector2d& T_fb, const std::deque<Eigen::Vector2d>& TB_fb,
                                const Eigen::Vector2d& tau_friction)
{
    // Return force exert to ground
    Eigen::Vector2d tau_inertia = inverseDynamic(TB_fb);

    // Eigen::Vector2d d_phi = dtb2dphi((TB_fb[0] - TB_fb[1]) / T_);
    // Eigen::Vector2d tau_friction = jointFriction(d_phi);
    // Eigen::Vector2d tau_friction = jointFriction(phi_vel);

    // Eigen::Vector2d F_est = jointTrq2footendForce(T_fb - tau_inertia - tau_friction, TB_fb[0]);
    // Eigen::Vector2d F_est = jointTrq2footendForce(T_fb - tau_friction, TB_fb[0]);

    Eigen::Vector2d F_est = jointTrq2footendForce(T_fb, TB_fb[0]);

    // term << T_fb[0] << "," << T_fb[1] << "," << tau_inertia[0] << "," << tau_inertia[1] << "," <<
    // tau_friction[0] << ","
    //      << tau_friction[1] << ",";

    return F_est;
}

Eigen::Vector2d forceEstimation(const Eigen::Vector2d& T_fb, const Eigen::Vector2d& TB, const Eigen::Vector2d& dTB,
                                const Eigen::Vector2d& ddTB, const Eigen::Vector2d& tau_friction)
{
    // Return force exert to ground
    // Eigen::Vector2d tau_inertia = inverseDynamic(TB, dTB, ddTB);
    // Eigen::Vector2d F_est = jointTrq2footendForce(T_fb - tau_inertia, TB);
    Eigen::Vector2d F_est = jointTrq2footendForce(T_fb, TB);
    return F_est;
}

Eigen::Vector2d inverseDynamic(const std::deque<Eigen::Vector2d>& TB)
{
    Eigen::Vector2d tb = TB[0];
    Eigen::Vector2d dtb = (TB[0] - TB[1]) / T_;
    Eigen::Vector2d dtb_1 = (TB[1] - TB[2]) / T_;
    Eigen::Vector2d ddtb = (dtb - dtb_1) / T_;

    /* q = [Rm; beta] */
    Eigen::Vector2d q(Rm(tb[0]), tb[1]);
    Eigen::Vector2d dq(dRm(tb[0], dtb[0]), dtb[1]);
    Eigen::Vector2d ddq(ddRm(tb[0], dtb[0], ddtb[0]), ddtb[1]);
    Eigen::Matrix2d Mq;
    Eigen::Vector2d Cq;
    Eigen::Vector2d Gq;

    Mq << leg_m, 0, 0, Ic(tb[0]) + leg_m * pow(q[0], 2);
    Cq << -leg_m * q[0] * pow(q[1], 2), 2 * leg_m * q[0] * dq[0] * dq[1] + dIc(tb[0], dtb[0]) * dq[1];
    Gq << -leg_m * g * cos(q[1]), -leg_m * g * q[0] * sin(q[1]);
    Eigen::Vector2d Frm_Tb;
    Eigen::Vector2d joint_trq;
    Frm_Tb = Mq * ddq + Cq + Gq;
    joint_trq = FrmTb2jointTrq(Frm_Tb, tb[0]);
    return joint_trq;
}

Eigen::Vector2d inverseDynamic(const Eigen::Vector2d& TB, const Eigen::Vector2d& dTB, const Eigen::Vector2d& ddTB)
{
    /* q = [Rm; beta] */
    Eigen::Vector2d q(Rm(TB[0]), TB[1]);
    Eigen::Vector2d dq(dRm(TB[0], dTB[0]), dTB[1]);
    Eigen::Vector2d ddq(ddRm(TB[0], dTB[0], ddTB[0]), ddTB[1]);
    Eigen::Matrix2d Mq;
    Eigen::Vector2d Cq;
    Eigen::Vector2d Gq;

    Mq << leg_m, 0, 0, Ic(TB[0]) + leg_m * pow(q[0], 2);
    Cq << -leg_m * q[0] * pow(q[1], 2), 2 * leg_m * q[0] * dq[0] * dq[1] + dIc(TB[0], dTB[0]) * dq[1];
    Gq << -leg_m * g * cos(q[1]), -leg_m * g * q[0] * sin(q[1]);
    Eigen::Vector2d Frm_Tb;
    Eigen::Vector2d joint_trq;
    Frm_Tb = Mq * ddq + Cq + Gq;
    joint_trq = FrmTb2jointTrq(Frm_Tb, TB[0]);
    return joint_trq;
}

Eigen::Vector2d adaptiveStiffness(const Eigen::Vector2d& F_err, const std::deque<Eigen::Vector2d>& pid_out,
                                  const std::deque<Eigen::Vector2d>& pid_err, const Eigen::Vector2d& kp,
                                  const Eigen::Vector2d& ki, const Eigen::Vector2d& kd)
{
    // F_err = F_leg2gnd_ref - F_leg2gnd_est (GRF error)
    Eigen::Vector2d k_stiffness(0, 0);
    double K_ADAPT_MAX = 2000000;
    double K_ADPAT_MIN = -5000;

    for (int i = 0; i < 2; i++)
    {
        double p_ = kp[i];
        double i_ = ki[i];
        double d_ = kd[i];
        double E_k = pid_err[0][i];
        double E_k_1 = pid_err[1][i];
        double E_k_2 = pid_err[2][i];
        double Y_k_2 = pid_out[2][i];
        double C2 = p_ + (i_ * T_ / 2) + (2 * d_ / T_);
        double C1 = i * T_ - 4 * d_ / T_;
        double C0 = -p_ + i_ * T_ / 2 + 2 * d_ / T_;
        double Y_k = C2 * E_k + C1 * E_k_1 + C0 * E_k_2;
        if (Y_k > K_ADAPT_MAX)
            k_stiffness[i] = K_ADAPT_MAX;
        else if (Y_k < K_ADPAT_MIN)
            k_stiffness[i] = K_ADPAT_MIN;
        else
            k_stiffness[i] = Y_k;
    }

    return k_stiffness;
}
