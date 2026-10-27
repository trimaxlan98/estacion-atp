#include "atp_control/mount_plant.hpp"

#include <algorithm>
#include <cmath>

namespace atp_control
{

namespace
{

double sgn(double x) {return (x > 0.0) - (x < 0.0);}   // np.sign (sgn(0) = 0)

double clip(double x, double lim) {return std::min(std::max(x, -lim), lim);}

// Un paso de velocidad de un cuerpo con friccion de Coulomb + Stribeck por el
// metodo de Karnopp (planta.py::_karnopp). Devuelve (w_nuevo, aceleracion efectiva).
struct Karnopp {double w; double a;};

Karnopp karnopp(
  double w, double neto, double J, double coulomb, double exceso, double vs,
  double veps, double h)
{
  const bool lento = std::fabs(w) < veps;
  const double fs = coulomb + exceso;
  const bool pega = lento && (std::fabs(neto) <= fs);
  const double signo = lento ? sgn(neto) : sgn(w);
  const double fric = stribeck_friction(w, coulomb, exceso, vs, signo);
  const double w_n = pega ? 0.0 : w + (neto - fric) / J * h;
  return {w_n, (w_n - w) / h};
}

}  // namespace

double stribeck_friction(double w, double coulomb, double exceso, double vs, double signo)
{
  const double x = w / vs;
  return (coulomb + exceso * std::exp(-(x * x))) * signo;
}

double quantize_angle(double theta_rad, int bits)
{
  if (bits <= 0) {
    return theta_rad;
  }
  const double q = 2.0 * M_PI / std::ldexp(1.0, bits);
  return q * std::nearbyint(theta_rad / q);
}

MountPlant::MountPlant(const PlantParams & p)
: p_(p)
{
  advanced_ = (p_.coulomb_nm > 0 || p_.stribeck_nm > 0 || p_.backlash_rad > 0 ||
    p_.mgd_nm != 0.0);
}

void MountPlant::engage(const Vec2 & theta0)
{
  theta_ = theta0;
  omega_ = {0.0, 0.0};
  theta_m_ = theta0;
  omega_m_ = {0.0, 0.0};
  lado_ = {0.0, 0.0};
  engaged_ = true;
}

const Vec2 & MountPlant::encoder_angle(bool motor_side) const
{
  if (motor_side && p_.backlash_rad > 0.0) {
    return theta_m_;
  }
  return theta_;
}

void MountPlant::advance(const Vec2 & u, const Vec2 & ext, double dt)
{
  // n subpasos de h y el resto (idem mount_sim.on_clock)
  const int n = static_cast<int>(std::floor(dt / p_.h + 1e-9));
  const double resto = dt - n * p_.h;
  for (int i = 0; i < n; ++i) {
    step(u, ext, p_.h);
  }
  if (resto > 1e-9) {
    step(u, ext, resto);
  }
}

void MountPlant::step(const Vec2 & u, const Vec2 & par_ext, double h)
{
  if (!advanced_) {
    // camino original, operacion por operacion (resultados identicos a planta.py)
    for (int i = 0; i < 2; ++i) {
      const double alpha = (u[i] + par_ext[i] - p_.b * omega_[i]) / p_.J;
      omega_[i] = clip(omega_[i] + alpha * h, p_.w_max);
      theta_[i] += omega_[i] * h;
    }
    return;
  }
  Vec2 ext = par_ext;
  if (p_.mgd_nm != 0.0) {
    ext[1] += -p_.mgd_nm * std::cos(theta_[1]);   // gravedad solo en elevacion
  }
  if (p_.backlash_rad > 0.0) {
    step_backlash(u, ext, h);
    return;
  }
  for (int i = 0; i < 2; ++i) {
    const double neto = u[i] + ext[i] - p_.b * omega_[i];
    const Karnopp k = karnopp(omega_[i], neto, p_.J, p_.coulomb_nm, p_.stribeck_nm,
        p_.v_stribeck, p_.v_eps, h);
    omega_[i] = clip(k.w, p_.w_max);
    theta_[i] += omega_[i] * h;
  }
}

// Modelo hibrido de dos cuerpos (Nordin y Gutman 2002): idem planta.py::_paso_backlash.
void MountPlant::step_backlash(const Vec2 & u, const Vec2 & ext, double h)
{
  const double J = p_.J, fm = p_.frac_motor;
  const double Jm = J * fm, Jl = J * (1.0 - fm);
  const double bm = p_.b * fm, bl = p_.b * (1.0 - fm);
  const double dh = 0.5 * p_.backlash_rad;
  const double e = p_.restitution;
  for (int i = 0; i < 2; ++i) {
    const double wl = omega_[i], wm = omega_m_[i];
    double lado = lado_[i];
    const bool cont = lado != 0.0;
    // cuerpo rigido (en contacto): inercia J, velocidad comun wl
    const double neto_r = u[i] + ext[i] - p_.b * wl;
    const Karnopp r = karnopp(wl, neto_r, J, p_.coulomb_nm, p_.stribeck_nm, p_.v_stribeck,
        p_.v_eps, h);
    const double tau_c = u[i] - bm * wl - Jm * r.a;      // par motor -> carga
    const bool sale = cont && (lado * tau_c < 0.0);
    // libre: el motor gira solo; la carga queda a su friccion y su par
    const double wm_f = wm + (u[i] - bm * wm) / Jm * h;
    const Karnopp lf = karnopp(wl, ext[i] - bl * wl, Jl, p_.coulomb_nm, p_.stribeck_nm,
        p_.v_stribeck, p_.v_eps, h);
    double wm_n = clip(cont ? r.w : wm_f, p_.w_max);
    double wl_n = clip(cont ? r.w : lf.w, p_.w_max);
    double thm = theta_m_[i] + wm_n * h;
    double thl = theta_[i] + wl_n * h;
    // en contacto el cuerpo es rigido: la carga queda a lado*holgura/2
    if (cont) {
      thl = thm - lado * dh;
    }
    // cierre de holgura entre cuerpos libres
    const double d = thm - thl;
    const bool golpe = (!cont) && (std::fabs(d) >= dh);
    if (golpe) {
      const double s = sgn(d);
      const double exceso = d - s * dh;
      thm = thm - exceso * Jl / J;
      thl = thl + exceso * Jm / J;
      const double vr = wm_n - wl_n;
      const bool cierra = s * vr > 0.0;
      const double vc = (Jm * wm_n + Jl * wl_n) / J;
      if (cierra) {
        const double wm_i = vc - e * Jl / J * vr;
        const double wl_i = vc + e * Jm / J * vr;
        wm_n = wm_i;
        wl_n = wl_i;
      }
      lado = s;
    }
    if (sale) {
      lado = 0.0;
    }
    theta_m_[i] = thm;
    theta_[i] = thl;
    omega_m_[i] = wm_n;
    omega_[i] = wl_n;
    lado_[i] = lado;
  }
}

}  // namespace atp_control
