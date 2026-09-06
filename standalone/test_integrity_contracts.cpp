// White-hat regression counterexamples, 2026-09-06.
// Extends cases independently observed in audit/2026-09-06-safety-contracts
// (92d2cd94). These are software contracts, not a railway safety certificate.
// Deliberately uses no assert(): checks must run in Release / NDEBUG too.
#include "tram_dr_localization/lin_alg.hpp"
#include "tram_dr_localization/ukf.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

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
      expect(e.confidence==Confidence::kOk,"estimator recovers when inputs are valid again");
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
      expect(e.confidence==Confidence::kOk,"controls recover without a manual reset");
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
    tram_dr::la::project_pd(a,n);
    double error=0;
    for(int i=0;i<n*n;++i) error=std::max(error,std::fabs(a[i]-original[i]));
    expect(error<1e-10,"PD repair preserves an already valid dense covariance");
    std::printf("OBS covariance_max_error=%.12g\n",error);
    double indef[]={1,2,2,1}, l[4];
    tram_dr::la::project_pd(indef,2);
    expect(tram_dr::la::chol(indef,l,2,0),"finite indefinite covariance is repaired");
  }
  std::printf("RESULT checks=%d failures=%d\n",checks,failures);
  return failures?1:0;
}
