#include "atp_control/laws.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <sstream>
#include <stdexcept>

namespace atp_control
{

// ---------------------------------------------------------------- algebra

Mat expm(const Mat & M, int terminos)
{
  const int n = static_cast<int>(M.rows());
  const double norma = std::max(M.cwiseAbs().rowwise().sum().maxCoeff(), 1e-30);
  const int s = std::max(0, static_cast<int>(std::ceil(std::log2(norma))) + 1);
  const Mat X = M / std::ldexp(1.0, s);
  Mat E = Mat::Identity(n, n);
  Mat T = Mat::Identity(n, n);
  for (int k = 1; k < terminos; ++k) {
    T = T * X / static_cast<double>(k);
    E = E + T;
  }
  for (int i = 0; i < s; ++i) {
    E = E * E;
  }
  return E;
}

void discretize_zoh(const Mat & A, const Mat & B, double Ts, Mat & Ad, Mat & Bd)
{
  const int n = static_cast<int>(A.rows()), m = static_cast<int>(B.cols());
  Mat M = Mat::Zero(n + m, n + m);
  M.topLeftCorner(n, n) = A;
  M.topRightCorner(n, m) = B;
  const Mat E = expm(M * Ts);
  Ad = E.topLeftCorner(n, n);
  Bd = E.topRightCorner(n, m);
}

void discretize_noise(const Mat & A, const Mat & Qc, double Ts, Mat & Ad, Mat & Qd)
{
  const int n = static_cast<int>(A.rows());
  Mat M = Mat::Zero(2 * n, 2 * n);
  M.topLeftCorner(n, n) = -A;
  M.topRightCorner(n, n) = Qc;
  M.bottomRightCorner(n, n) = A.transpose();
  const Mat E = expm(M * Ts);
  Ad = E.bottomRightCorner(n, n).transpose();
  const Mat q = Ad * E.topRightCorner(n, n);
  Qd = (q + q.transpose()) / 2.0;
}

Mat dare(const Mat & A, const Mat & B, const Mat & Q, const Mat & R, double tol, int max_iter)
{
  const int n = static_cast<int>(A.rows());
  const Mat I = Mat::Identity(n, n);
  Mat Ak = A, Gk = B * R.inverse() * B.transpose(), Hk = Q;
  for (int it = 0; it < max_iter; ++it) {
    const Mat W = (I + Gk * Hk).inverse();
    const Mat An = Ak * W * Ak;
    const Mat Gn = Gk + Ak * W * Gk * Ak.transpose();
    const Mat Hn = Hk + Ak.transpose() * Hk * W * Ak;
    const double dif = (Hn - Hk).cwiseAbs().maxCoeff();
    Ak = An;
    Gk = (Gn + Gn.transpose()) / 2.0;
    Hk = (Hn + Hn.transpose()) / 2.0;
    if (dif <= tol * std::max(1.0, Hk.cwiseAbs().maxCoeff())) {
      return Hk;
    }
  }
  throw std::runtime_error("dare: no converge");
}

LqrDesign lqr_integral(
  double J, double b, double Ts, double q_theta, double q_omega, double q_int, double r)
{
  Mat A(2, 2), B(2, 1);
  A << 0.0, 1.0, 0.0, -b / J;
  B << 0.0, 1.0 / J;
  Mat Ad, Bd;
  discretize_zoh(A, B, Ts, Ad, Bd);
  // estado aumentado z = [theta, omega, zeta], zeta+ = zeta + Ts theta
  Mat Aa = Mat::Zero(3, 3), Ba = Mat::Zero(3, 1);
  Aa.topLeftCorner(2, 2) = Ad;
  Aa(2, 0) = Ts;
  Aa(2, 2) = 1.0;
  Ba.topRows(2) = Bd;
  Mat Q = Mat::Zero(3, 3);
  Q.diagonal() << q_theta, q_omega, q_int;
  Mat R(1, 1);
  R << r;
  const Mat X = dare(Aa, Ba, Q, R);
  const Mat K = (R + Ba.transpose() * X * Ba).ldlt().solve(Ba.transpose() * X * Aa);
  LqrDesign d;
  d.K = K.row(0);
  d.Acl = Aa - Ba * K;
  d.Ts = Ts;
  const Eigen::EigenSolver<Mat> es(d.Acl, false);
  for (int i = 0; i < 3; ++i) {
    d.max_pole_abs = std::max(d.max_pole_abs, std::abs(es.eigenvalues()(i)));
  }
  return d;
}

KalmanDesign kalman_design(
  double J, double b, double Ts, double sigma_med_rad, int bits, double q_acc, double q_par)
{
  Mat A(3, 3), B(3, 1), Qc = Mat::Zero(3, 3);
  A << 0.0, 1.0, 0.0, 0.0, -b / J, 1.0 / J, 0.0, 0.0, 0.0;
  B << 0.0, 1.0 / J, 0.0;
  Qc.diagonal() << 0.0, q_acc, q_par;
  Mat Ad, Bd, Ad2, Qd;
  discretize_zoh(A, B, Ts, Ad, Bd);
  discretize_noise(A, Qc, Ts, Ad2, Qd);
  const double q = bits > 0 ? 2.0 * M_PI / std::ldexp(1.0, bits) : 0.0;
  // piso de 1e-6 rad: con ruido y bits nulos R = 0 haria al filtro ciego a su modelo
  const double s = std::max(sigma_med_rad, 1e-6);
  KalmanDesign d;
  d.Ad = Ad;
  d.Bd = Bd.col(0);
  d.Qd = Qd;
  d.R = s * s + q * q / 12.0;
  return d;
}

// ---------------------------------------------------------------- bloques

void RefDifferentiator::step(const Vec2 & r, Vec2 & v, Vec2 & a)
{
  for (int i = 0; i < 2; ++i) {
    if (n_ >= 2) {
      v[i] = (3.0 * r[i] - 4.0 * r1_[i] + r2_[i]) / (2.0 * Ts_);
      a[i] = (r[i] - 2.0 * r1_[i] + r2_[i]) / (Ts_ * Ts_);
    } else {
      v[i] = 0.0;
      a[i] = 0.0;
    }
  }
  r2_ = r1_;
  r1_ = r;
  ++n_;
}

namespace
{

// Sin Kalman: theta_h = medida, omega_h = derivada filtrada de 1.er orden (tau 0.08 s).
class DerivativeEstimator : public Estimator
{
public:
  DerivativeEstimator(const Vec2 & y0, double Ts, double tau)
  : Ts_(Ts), alfa_(Ts / (tau + Ts)), y_prev_(y0)
  {
    x_.th = y0;
  }
  Estimate step(const Vec2 & y, const Vec2 &) override
  {
    for (int i = 0; i < 2; ++i) {
      w_[i] = w_[i] + alfa_ * ((y[i] - y_prev_[i]) / Ts_ - w_[i]);
      y_prev_[i] = y[i];
    }
    x_.th = y;
    x_.w = w_;
    x_.d = {0.0, 0.0};
    return x_;
  }
  Estimate initial() const override {return x_;}

private:
  double Ts_, alfa_;
  Vec2 y_prev_, w_{0, 0};
  Estimate x_;
};

// Kalman discreto de [theta, omega, d] por eje; la covarianza P es COMUN a los
// dos ejes (mismo modelo y misma R), se propaga una vez (FiltroKalman). Joseph.
class KalmanEstimator : public Estimator
{
public:
  KalmanEstimator(const KalmanDesign & d, const Vec2 & y0)
  : d_(d)
  {
    P_ = Eigen::Matrix3d::Zero();
    P_.diagonal() << 2e-3 * 2e-3, 0.05 * 0.05, 0.1 * 0.1;
    for (int i = 0; i < 2; ++i) {
      x_[i] = Eigen::Vector3d(y0[i], 0.0, 0.0);
    }
  }
  Estimate step(const Vec2 & y, const Vec2 & u_prev) override
  {
    for (int i = 0; i < 2; ++i) {
      x_[i] = d_.Ad * x_[i] + d_.Bd * u_prev[i];
    }
    P_ = d_.Ad * P_ * d_.Ad.transpose() + d_.Qd;
    const double S = P_(0, 0) + d_.R;
    const Eigen::Vector3d K = P_.col(0) / S;
    for (int i = 0; i < 2; ++i) {
      x_[i] = x_[i] + K * (y[i] - x_[i](0));
    }
    Eigen::Matrix3d IKH = Eigen::Matrix3d::Identity();
    IKH.col(0) -= K;
    P_ = IKH * P_ * IKH.transpose() + d_.R * (K * K.transpose());
    P_ = (P_ + P_.transpose()) / 2.0;
    return initial();
  }
  Estimate initial() const override
  {
    Estimate e;
    for (int i = 0; i < 2; ++i) {
      e.th[i] = x_[i](0);
      e.w[i] = x_[i](1);
      e.d[i] = x_[i](2);
    }
    return e;
  }

private:
  KalmanDesign d_;
  Eigen::Matrix3d P_;
  Eigen::Vector3d x_[2];
};

double clipd(double x, double lim) {return std::min(std::max(x, -lim), lim);}

// ---------------------------------------------------------------- PID

class PidLaw : public TrackingLaw
{
public:
  explicit PidLaw(const LawParams & p)
  : p_(p), dt_(1.0 / p.rate_hz), dif_(1.0 / p.rate_hz)
  {
    kalman_ = p.estimator == "kalman";
  }
  Vec2 compute(const Vec2 & meas, const Vec2 & ref) override
  {
    Vec2 e{0, 0};
    if (kalman_) {
      // error y derivada del error desde el estado estimado
      Estimate x;
      if (!est_) {
        est_ = make_estimator(true, p_.J, p_.b, dt_, meas, sigma(), p_.enc_bits,
            p_.kf_q_acc, p_.kf_q_par);
        x = est_->initial();
      } else {
        x = est_->step(meas, u_);
      }
      Vec2 rv, ra;
      dif_.step(ref, rv, ra);
      for (int i = 0; i < 2; ++i) {
        e[i] = ref[i] - x.th[i];
        d_filt_[i] = rv[i] - x.w[i];       // ya filtrada por el Kalman
      }
    } else {
      const double alfa = dt_ / (p_.tau_derivada + dt_);
      for (int i = 0; i < 2; ++i) {
        e[i] = ref[i] - meas[i];
        const double d_bruta = (e[i] - e_prev_[i]) / dt_;
        d_filt_[i] += alfa * (d_bruta - d_filt_[i]);
      }
    }
    for (int i = 0; i < 2; ++i) {
      const double u_raw = p_.kp * e[i] + p_.ki * integral_[i] + p_.kd * d_filt_[i];
      u_[i] = clipd(u_raw, p_.u_max);
      const bool sat = std::fabs(u_raw) > p_.u_max + 1e-12;
      // clamping: no integra si el mando esta saturado y el error empuja mas
      const bool frozen = p_.antiwindup && sat && (u_raw * e[i] > 0.0);
      integral_[i] += frozen ? 0.0 : e[i] * dt_;
      e_prev_[i] = e[i];
    }
    return u_;
  }
  std::string describe() const override
  {
    std::ostringstream s;
    s << "PID kp=" << p_.kp << " ki=" << p_.ki << " kd=" << p_.kd << " a "
      << p_.rate_hz << " Hz | estimador " << (kalman_ ? "kalman" : "derivada filtrada");
    return s.str();
  }

private:
  double sigma() const {return p_.enc_sigma_deg * M_PI / 180.0;}
  LawParams p_;
  double dt_;
  bool kalman_ = false;
  RefDifferentiator dif_;
  std::unique_ptr<Estimator> est_;
  Vec2 u_{0, 0}, integral_{0, 0}, e_prev_{0, 0}, d_filt_{0, 0};
};

// ---------------------------------------------------------------- LQR

class LqrLaw : public TrackingLaw
{
public:
  explicit LqrLaw(const LawParams & p)
  : p_(p), dt_(1.0 / p.rate_hz), dif_(1.0 / p.rate_hz)
  {
    kalman_ = p.estimator == "kalman";
    dis_ = lqr_integral(p.J, p.b, dt_, p.q_theta, p.q_omega, p.q_int, p.r_u);
    dhat_ = kalman_ && p.use_dhat;
  }
  Vec2 compute(const Vec2 & meas, const Vec2 & ref) override
  {
    Estimate x;
    if (!est_) {
      est_ = make_estimator(kalman_, p_.J, p_.b, dt_, meas, p_.enc_sigma_deg * M_PI / 180.0,
          p_.enc_bits, p_.kf_q_acc, p_.kf_q_par);
      x = est_->initial();
    } else {
      x = est_->step(meas, u_);
    }
    Vec2 rv, ra;
    dif_.step(ref, rv, ra);
    // LazoLQR.calcular
    const auto & K = dis_.K;
    for (int i = 0; i < 2; ++i) {
      const double e_p = x.th[i] - ref[i];
      const double e_v = x.w[i] - (p_.feedforward ? rv[i] : 0.0);
      const double u_ff = p_.feedforward ? (p_.J * ra[i] + p_.b * rv[i]) : 0.0;
      double u_raw = u_ff - (K(0) * e_p + K(1) * e_v + K(2) * zeta_[i]);
      if (dhat_) {
        u_raw = u_raw - x.d[i];
      }
      u_[i] = clipd(u_raw, p_.u_max);
      const bool sat = std::fabs(u_raw) > p_.u_max + 1e-12;
      const bool congelar = p_.antiwindup && sat && (u_raw * (-e_p) > 0.0);
      zeta_[i] += congelar ? 0.0 : e_p * dt_;
    }
    return u_;
  }
  std::string describe() const override
  {
    std::ostringstream s;
    s << "LQR integral a " << p_.rate_hz << " Hz: K=[" << dis_.K(0) << ", " << dis_.K(1)
      << ", " << dis_.K(2) << "] |z|max " << dis_.max_pole_abs << " | estimador "
      << (kalman_ ? "kalman" : "derivada filtrada");
    return s.str();
  }

private:
  LawParams p_;
  double dt_;
  bool kalman_ = false, dhat_ = false;
  LqrDesign dis_;
  RefDifferentiator dif_;
  std::unique_ptr<Estimator> est_;
  Vec2 u_{0, 0}, zeta_{0, 0};
};

}  // namespace

std::unique_ptr<Estimator> make_estimator(
  bool kalman, double J, double b, double Ts, const Vec2 & y0, double sigma_med_rad, int bits,
  double q_acc, double q_par)
{
  if (kalman) {
    return std::make_unique<KalmanEstimator>(
      kalman_design(J, b, Ts, sigma_med_rad, bits, q_acc, q_par), y0);
  }
  return std::make_unique<DerivativeEstimator>(y0, Ts, 0.08);
}

std::unique_ptr<TrackingLaw> make_law(const LawParams & p)
{
  if (p.estimator != "none" && p.estimator != "kalman") {
    throw std::invalid_argument("estimator desconocido: " + p.estimator);
  }
  if (p.law == "pid") {
    return std::make_unique<PidLaw>(p);
  }
  if (p.law == "lqr") {
    return std::make_unique<LqrLaw>(p);
  }
  throw std::invalid_argument("ley desconocida: " + p.law + " (pid | lqr)");
}

}  // namespace atp_control
