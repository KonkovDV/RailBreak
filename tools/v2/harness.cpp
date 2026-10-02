// Core harness: bogies, notch, GNSS start and burst snaps through TrackOdometer.
// events: t,kind,value  kind 0 front,1 rear,2 cmd,3 snap(value=s at t_aux),4 init(value=s0); col4 t_aux
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <array>
#include "railbreak_backup_odometry/track_odometer.hpp"
#include "railbreak_backup_odometry/map_match.hpp"
int main(int argc,char**argv){
  if(argc<4){std::cerr<<"harness assets events out [sigma_k0] [stop_anchor 0/1]\n";return 2;}
  railbreak::Assets A=railbreak::load_assets(argv[1]);
  railbreak::Params p; p.k0=A.k0;
  for(int i=4;i<argc;++i){ std::string a=argv[i]; auto e=a.find('='); std::string k=a.substr(0,e); double v=std::stod(a.substr(e+1));
    if(k=="sigma_k0")p.sigma_k0=v; else if(k=="stop_gate")p.stop_gate=v; else if(k=="stop_sd_max")p.stop_sd_max=v; else if(k=="q_k")p.q_k=v; else if(k=="q_v")p.q_v=v; else {std::cerr<<"bad "<<k<<"\n";return 3;} }
  railbreak::TrackOdometer od(&A,p);
  std::ifstream in(argv[2]); std::ofstream out(argv[3]); out.setf(std::ios::fixed); out.precision(6);
  out<<"t,kind,s,v,k,sigma_s\n";
  std::string line; std::getline(in,line);
  std::deque<std::array<double,2>> hist; bool inited=false;
  while(std::getline(in,line)){
    std::stringstream ss(line); std::string c; double f[4]={0,0,0,0};
    for(int i=0;i<4&&std::getline(ss,c,',');++i) f[i]=std::stod(c);
    double t=f[0]; int kind=(int)f[1];
    if(kind==4){ od.init(f[2],1.0); od.set_time(t); inited=true; continue;}
    if(!inited) continue;
    if(kind==3){
      // carry by arc travelled since t_aux
      double s_then=od.s();
      for(size_t i=1;i<hist.size();++i){ if(hist[i-1][0]<=f[3]&&f[3]<=hist[i][0]){
          double a=(f[3]-hist[i-1][0])/std::max(1e-9,hist[i][0]-hist[i-1][0]);
          double ds=railbreak::arc_delta(A.map,hist[i][1],hist[i-1][1]);
          s_then=A.map.wrap(hist[i-1][1]+a*ds); break;} }
      double carried=railbreak::arc_delta(A.map,od.s(),s_then);
      od.gnss_snap(A.map.wrap(f[2]+carried),0.5); continue;}
    if(kind==2) od.on_cmd(t,(int)f[2]); else od.on_bogie(t,kind==0,f[2]);
    hist.push_back({t,od.s()}); while(hist.size()>2&&t-hist.front()[0]>30) hist.pop_front();
    out<<t<<','<<kind<<','<<od.s()<<','<<od.v()<<','<<od.k()<<','<<od.sigma_s()<<'\n';
  }
}
