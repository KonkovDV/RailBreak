// Software contracts, not a railway safety certificate.
// Deliberately uses no assert(): checks must run in Release / NDEBUG too.
#include "tram_dr_localization/lin_alg.hpp"
#include "tram_dr_localization/ukf.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace {
int checks = 0, failures = 0;
void expect(bool condition, const char* name) {
  ++checks;
  if (!condition) ++failures;
  std::printf("%s %s\n", condition ? "PASS" : "FAIL", name);
}
tram_dr::UkfEstimate warm(tram_dr::Ukf& f, tram_dr::Input u = {}) {
  const double w[] = {5.0/.35, 5.0/.35, 5.0/.35, 5.0/.35};
  tram_dr::UkfEstimate e;
  for (int k=0; k<50; ++k) e=f.predict_and_update(u,w,4,.02);
  return e;
}
bool finite(const tram_dr::UkfEstimate& e) {
  return std::isfinite(e.x.s_m) && std::isfinite(e.x.v_mps) &&
         std::isfinite(e.x.a_mps2) && std::isfinite(e.x.f_bias_n) &&
         std::isfinite(e.x.m_eff_kg) && std::isfinite(e.p_ss) &&
         std::isfinite(e.p_vv) && e.p_ss>=0 && e.p_vv>=0;
}
}
int main() {
  using tram_dr::Confidence; using tram_dr::Mode;
  const double zero[]={0,0,0,0};
  const double w[]={5/.35,5/.35,5/.35,5/.35};
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  {
    tram_dr::Ukf f; tram_dr::Input u; u.brake=.3;
    tram_dr::UkfEstimate e;
    for (int k=0;k<150;++k) e=f.predict_and_update(u,zero,4,.02);
    expect(e.mode==Mode::kStandstill,"fresh zeros enter standstill");
    expect(e.n_omega_used==4,"ZUPT counts this packet's four valid encoders");
    expect(e.confidence!=Confidence::kLost,"fresh ZUPT is not an encoder outage");
    expect(!e.nis_valid && e.nis==0,"ZUPT is not a rolling update");
    u.brake=0; u.notch=.5;
    e=f.predict_and_update(u,w,4,.02);
    expect(e.mode!=Mode::kStandstill,"departure clears standstill mode");
  }
  {
    tram_dr::Ukf f; tram_dr::Input u;
    auto e=warm(f);
    expect(e.confidence==Confidence::kOk,"healthy moving precondition");
    for (int k=0;k<15;++k) e=f.predict_and_update(u,nullptr,0,.02);
    expect(e.n_omega_used==0,"predict-only never reuses the old wheel count");
    expect(!e.nis_valid && e.nis==0,"predict-only resets measurement diagnostics");
    expect(e.confidence!=Confidence::kOk,"missing packets degrade aggregate");
    expect(e.confidence_v!=Confidence::kOk,"missing packets degrade speed");
    expect(e.confidence_s!=Confidence::kOk,"missing packets degrade position");
    for(int k=0;k<50;++k) e=f.predict_and_update(u,nullptr,0,.02);
    expect(e.confidence==Confidence::kLost,"silence reaches LOST by elapsed time");
    e=f.predict_and_update(u,w,4,.02);
    expect(e.n_omega_used==4 && finite(e),"fresh packet recovers usable wheel evidence");
  }
  {
    tram_dr::Ukf f; tram_dr::Input u; warm(f);
    tram_dr::UkfEstimate e;
    for(int k=0;k<65;++k) e=f.predict_and_update(u,nullptr,4,.02);
    expect(finite(e),"null pointer with nonzero count is safe");
    expect(e.confidence==Confidence::kLost,"null pointer cannot refresh wheel age");
  }
  {
    tram_dr::Ukf f; tram_dr::Input u; u.brake=.3;
    for(int k=0;k<30;++k) f.predict_and_update(u,zero,4,.02);
    auto e=f.predict_and_update(u,nullptr,0,.02);
    expect(e.mode!=Mode::kStandstill,"standstill is not held without fresh evidence");
    const double partial[]={0,nan,nan,nan};
    e=f.predict_and_update(u,partial,4,.02);
    expect(e.mode!=Mode::kStandstill,"partial invalid packet cannot prove standstill");
  }
  {
    tram_dr::Ukf f; tram_dr::Input u; u.brake=.3;
    const double bad[]={nan,nan,nan,nan};
    f.predict_and_update(u,zero,4,.02);
    auto e=f.predict_and_update(u,bad,4,.02);
    expect(e.confidence==Confidence::kLost,"four invalid encoders fail closed");
    for(int k=0;k<100;++k) e=f.predict_and_update(u,zero,4,.02);
    expect(e.confidence!=Confidence::kLost,"fresh stationary packets clear old outage");
    expect(e.sca.n_inflated==0,"ZUPT refreshes stale SCA fault flags");
  }
  {
    tram_dr::UkfParams p; p.r0_uncalibrated=true; tram_dr::Ukf f(p);
    auto e=warm(f);
    expect(e.confidence_s==Confidence::kDegraded,"unknown radius degrades path scale");
    expect(e.confidence_v==Confidence::kDegraded,"unknown radius also degrades speed scale");
  }
  {
    tram_dr::UkfParams p; p.p_ss_init=25; tram_dr::Ukf f(p);
    auto e=warm(f);
    expect(e.pl_s_m>=e.al_s_m,"PL alert precondition");
    expect(e.confidence_s!=Confidence::kOk,"PL_s alert belongs to position status");
    expect(!(e.confidence==Confidence::kDegraded && e.confidence_s==Confidence::kOk && e.confidence_v==Confidence::kOk),"aggregate agrees with channel statuses");
  }
  {
    tram_dr::Ukf f; tram_dr::Input u; u.notch=.4; warm(f,u);
    const double spin[]={12/.35,12/.35,12/.35,12/.35}; tram_dr::UkfEstimate e;
    for(int k=0;k<15;++k) e=f.predict_and_update(u,spin,4,.02);
    expect(e.sca.common_mode,"common-mode slip precondition");
    expect(e.confidence_v!=Confidence::kOk,"active common slip cannot claim OK speed");
  }
  {
    tram_dr::Ukf f; tram_dr::Input u; warm(f);
    tram_dr::UkfEstimate e;
    for(int k=0;k<16;++k) e=f.predict_and_update(u,zero,4,.02);
    expect(e.mode!=Mode::kStandstill,"coasting locked wheels are not ZUPT");
    expect(e.x.v_mps>1,"false zero wheels cannot erase a moving body");
    expect(e.confidence!=Confidence::kOk,"coasting lock cannot claim OK");
    std::printf("OBS coasting_zero_v=%.9g\n",e.x.v_mps);
  }
  {
    tram_dr::Ukf f; tram_dr::Input u; warm(f); u.notch_valid=false;
    auto e=f.predict_and_update(u,w,4,.02);
    expect(e.confidence!=Confidence::kOk,"unavailable control degrades before LOST timeout");
    expect(e.confidence_v!=Confidence::kOk,"unavailable control degrades speed channel");
  }
  {
    const double bad_dt[]={0,-.02,nan,inf,1.0};
    for(double dt:bad_dt) {
      tram_dr::Ukf f; tram_dr::Input u; auto before=warm(f);
      auto e=f.predict_and_update(u,w,4,dt);
      expect(finite(e),"invalid dt retains finite last estimate");
      expect(e.confidence==Confidence::kLost,"invalid dt reports LOST");
      expect(e.x.s_m==before.x.s_m,"rejected dt does not propagate state");
      e=f.predict_and_update(u,w,4,.02);
      expect(e.path_integrity_latched,"rejected dt latches path");
      expect(e.confidence!=Confidence::kLost,"next valid dt is not stuck LOST");
      expect(e.confidence!=Confidence::kOk,"recovered v is not recovered s");
    }
    for(int field=0;field<2;++field) {
      tram_dr::Ukf f; tram_dr::Input u; auto before=warm(f);
      if(field==0) u.notch=nan; else u.brake=inf;
      auto e=f.predict_and_update(u,w,4,.02);
      expect(finite(e),"nonfinite controls cannot poison output");
      expect(e.confidence==Confidence::kLost,"nonfinite controls report LOST");
      expect(e.x.s_m==before.x.s_m,"rejected controls do not propagate state");
      u.notch=0; u.brake=0;
      e=f.predict_and_update(u,w,4,.02);
      expect(e.path_integrity_latched,"rejected controls latch path");
      expect(e.confidence!=Confidence::kLost,"next valid command is not stuck LOST");
      expect(e.confidence!=Confidence::kOk,"OK after reject would hide lost path");
    }
    {
      tram_dr::Ukf f; tram_dr::Input u; auto before=warm(f);
      u.notch_valid=false; u.notch=nan;
      auto e=f.predict_and_update(u,w,4,.02);
      expect(finite(e),"invalid NaN notch is coast, not a reject");
      expect(e.x.s_m!=before.x.s_m,"invalid NaN notch still integrates");
      expect(e.confidence!=Confidence::kLost,"invalid NaN notch is not LOST on the first frame");
      expect(e.confidence!=Confidence::kOk,"invalid notch cannot claim OK");
    }
  }
  {
    for(double x:{nan,inf}) {
      double a[]={x,0,0,1}, l[4];
      expect(!tram_dr::la::chol(a,l,2,0),"Cholesky rejects nonfinite covariance");
    }
    constexpr int n=tram_dr::kStateDim;
    double a[n*n], original[n*n];
    for(int i=0;i<n;++i) for(int j=0;j<n;++j) original[i*n+j]=a[i*n+j]=i==j?20+i:.01*(i+j+1);
    expect(tram_dr::la::project_pd(a,n),"PD repair reports success on a valid dense covariance");
    double error=0;
    for(int i=0;i<n*n;++i) error=std::max(error,std::fabs(a[i]-original[i]));
    expect(error<1e-10,"PD repair preserves an already valid dense covariance");
    std::printf("OBS covariance_max_error=%.12g\n",error);
    double indef[]={1,2,2,1}, l[4];
    expect(tram_dr::la::project_pd(indef,2),"PD repair reports success on a finite indefinite matrix");
    expect(tram_dr::la::chol(indef,l,2,0),"finite indefinite covariance is repaired");
    double poisoned[]={nan,0,0,1};
    expect(!tram_dr::la::project_pd(poisoned,2),"project_pd refuses a nonfinite matrix");
    expect(std::isnan(poisoned[0]) && poisoned[1]==0 && poisoned[2]==0 && poisoned[3]==1,
           "project_pd leaves a poisoned matrix byte-stable");
  }
  {
    tram_dr::UkfParams p; p.beta=0.0;
    bool threw=false;
    try { tram_dr::Ukf f(p); } catch (const std::invalid_argument&) { threw=true; }
    expect(threw,"validate_ukf rejects alpha=0.58 at beta=0");
    p.beta=1.0;
    threw=false;
    try { tram_dr::Ukf f(p); } catch (const std::invalid_argument&) { threw=true; }
    expect(threw,"validate_ukf rejects alpha=0.58 at beta=1");
    p.beta=2.0;
    threw=false;
    try { tram_dr::Ukf f(p); } catch (...) { threw=true; }
    expect(!threw,"shipped alpha=0.58, beta=2 remains admissible");
    p.beta=0.0; p.cubature=true;
    threw=false;
    try { tram_dr::Ukf f(p); } catch (...) { threw=true; }
    expect(!threw,"cubature mode does not use the scaled-UT weight window");
  }
  {
    tram_dr::Ukf f;
    tram_dr::ScaParams sca;
    f.set_sca(sca);
    tram_dr::PlantParams plant;
    plant.r0_m = 0.40;
    bool threw = false;
    try {
      f.set_plant(plant);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    expect(threw, "set_plant throws after set_sca when r0 disagrees");
    plant.r0_m = 0.35;
    threw = false;
    try {
      f.set_plant(plant);
    } catch (...) {
      threw = true;
    }
    expect(!threw, "set_plant agrees with locked SCA r0");
  }
  {
    tram_dr::Ukf f;
    tram_dr::PlantParams plant;
    plant.r0_m = 0.40;
    bool threw = false;
    try {
      f.set_plant(plant);
    } catch (...) {
      threw = true;
    }
    expect(!threw, "set_plant alone still syncs SCA r0");
    tram_dr::ScaParams sca;
    sca.r0_m = 0.40;
    threw = false;
    try {
      f.set_sca(sca);
    } catch (...) {
      threw = true;
    }
    expect(!threw, "set_sca after matching set_plant is allowed");
  }
  {
    tram_dr::Ukf f;
    tram_dr::Input u;
    auto e = warm(f);
    for (int k = 0; k < 16; ++k) e = f.predict_and_update(u, zero, 4, .02);
    expect(e.mode != Mode::kStandstill, "short coasting lock is still not ZUPT");
    expect(!e.zupt_forced, "sub-two-second zeros are not a forced ZUPT");
    for (int k = 0; k < 120; ++k) e = f.predict_and_update(u, zero, 4, .02);
    expect(e.mode == Mode::kStandstill, "omega-only ZUPT after 2 s of rest");
    expect(e.zupt_forced, "omega-only ZUPT is flagged, not silent");
    expect(e.confidence != Confidence::kOk, "forced ZUPT degrades integrity");
    expect(std::fabs(e.x.v_mps) < 1e-9, "forced ZUPT still zeros v");
  }
  {
    tram_dr::Ukf f;
    tram_dr::Input u;
    auto e = warm(f);
    for (int k = 0; k < 20; ++k) e = f.predict_and_update(u, nullptr, 0, .02);
    expect(!e.sca_current, "predict-only does not advertise a live SCA picture");
  }
  {
    tram_dr::Ukf f;
    tram_dr::Input u;
    u.notch = .4;
    warm(f, u);
    const double spin[] = {12 / .35, 12 / .35, 12 / .35, 12 / .35};
    tram_dr::UkfEstimate e;
    for (int k = 0; k < 15; ++k) e = f.predict_and_update(u, spin, 4, .02);
    expect(e.s_unbounded, "slip latch publishes unbounded path, not 1e6 m");
    expect(e.over_m == 0 && e.b_s_m == 0, "over_m is not a million-metre sentinel");
    expect(!std::isfinite(e.pl_s_m), "PL is infinite while s is unbounded");
  }
  {
    tram_dr::Ukf f;
    tram_dr::Input u;
    u.brake = .3;
    tram_dr::UkfEstimate e;
    const double one[] = {0};
    for (int k = 0; k < 150; ++k) e = f.predict_and_update(u, one, 1, .02);
    expect(e.mode != Mode::kStandstill, "one zero encoder cannot prove standstill");
  }
  {
    tram_dr::Ukf f;
    tram_dr::Input u;
    auto e = warm(f);
    const double one[] = {5 / .35};
    e = f.predict_and_update(u, one, 1, .02);
    expect(e.confidence != Confidence::kOk, "truncated wheel packet degrades");
  }
  {
    tram_dr::Ukf a;
    tram_dr::Ukf b;
    tram_dr::Input u;
    u.notch = .8;
    warm(a, u);
    warm(b, u);
    for (int k = 0; k < 10; ++k) {
      a.predict_and_update(u, w, 4, .02);
      b.predict_and_update(u, w, 4, .02);
    }
    tram_dr::Input dead = u;
    dead.notch_valid = false;
    tram_dr::UkfEstimate ea, eb;
    for (int k = 0; k < 80; ++k) {
      ea = a.predict_and_update(u, nullptr, 0, .02);
      eb = b.predict_and_update(dead, nullptr, 0, .02);
    }
    expect(eb.x.v_mps < ea.x.v_mps - 0.2, "invalid notch must not keep applying traction");
  }
  {
    tram_dr::Ukf f; tram_dr::Input u;
    auto e=warm(f);
    expect(e.confidence==Confidence::kOk,"healthy moving precondition");
    for(int k=0;k<500;++k) e=f.predict_and_update(u,zero,4,.02);
    expect(e.path_integrity_latched,"F-10 at speed latches path integrity");
    for(int k=0;k<40;++k) e=f.predict_and_update(u,w,4,.02);
    expect(e.path_integrity_latched,"path latch survives wheel recovery");
    expect(e.confidence!=Confidence::kOk,"recovered v is not HMI-OK on s");
  }
  {
    tram_dr::Ukf f; tram_dr::Input u; warm(f);
    auto e=f.predict_and_update(u,w,4,nan);
    expect(e.confidence==Confidence::kLost,"NaN dt is LOST this frame");
    expect(e.path_integrity_latched,"rejected dt latches path");
    e=f.predict_and_update(u,w,4,.02);
    expect(e.path_integrity_latched,"next valid frame keeps the latch");
    expect(e.confidence!=Confidence::kOk,"OK after reject would hide lost path");
  }
  {
    double huge[]={1e308,0,0,1e308};
    double orig[]={1e308,0,0,1e308};
    const bool ok=tram_dr::la::project_pd(huge,2);
    if(!ok){
      expect(huge[0]==orig[0] && huge[1]==orig[1] && huge[2]==orig[2] && huge[3]==orig[3],
             "project_pd false restores the input");
    }
  }
  std::printf("RESULT checks=%d failures=%d\n",checks,failures);
  return failures?1:0;
}
