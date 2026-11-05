#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif
#include "../../../control/drive.hpp"

namespace {
double monotonic() {
#ifdef __APPLE__
  // CPython's time.monotonic uses mach_absolute_time, whereas libc++'s
  // steady_clock includes a different suspension epoch on this platform.
  static mach_timebase_info_data_t base = [] { mach_timebase_info_data_t v; mach_timebase_info(&v); return v; }();
  return mach_absolute_time() * (double(base.numer)/base.denom) / 1e9;
#else
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}
double number(const std::string &s, const std::string &key, double fallback) {
  auto n=s.find("\""+key+"\""); if(n==std::string::npos) return fallback;
  n=s.find(':', n); if(n==std::string::npos) return fallback;
  char *end=nullptr; const char *start=s.c_str()+n+1;
  double value=std::strtod(start,&end);
  return end!=start && std::isfinite(value) ? value : fallback;
}
std::string word(const std::string &s,const std::string &key) {
  auto n=s.find("\""+key+"\""); if(n==std::string::npos)return {};
  n=s.find(':',n); n=s.find('"',n); if(n==std::string::npos)return {};
  const auto e=s.find('"',n+1); return e==std::string::npos?"":s.substr(n+1,e-n-1);
}
int port(const char *key,int fallback) { const char *v=std::getenv(key); return v?std::atoi(v):fallback; }
}

int main() {
  webots::Robot robot;
  robotracers::Parameters params;
  if(const char *root=std::getenv("ROBOTRACERS_ROOT")) {
    std::ifstream f(std::string(root)+"/simulation/config/robot.json");
    std::stringstream ss;ss<<f.rdbuf();const auto s=ss.str();
    params.radius=number(s,"wheel_radius_m",params.radius);
    params.track=number(s,"track_width_m",params.track);
    params.counts_per_turn=number(s,"encoder_counts_per_revolution",params.counts_per_turn);
    params.voltage=number(s,"motor_voltage",params.voltage);
    params.no_load_rpm=number(s,"motor_no_load_rpm",params.no_load_rpm);
    params.no_load_current=number(s,"motor_no_load_current_a",params.no_load_current);
    params.stall_current=number(s,"motor_stall_current_a",params.stall_current);
    params.stall_torque=number(s,"motor_stall_torque_nm",params.stall_torque);
    params.max_current=number(s,"motor_current_limit_a",params.max_current);
  }
  robotracers::Drive drive(params);
  auto *left=robot.getMotor("left_motor"), *right=robot.getMotor("right_motor");
  auto *le=robot.getPositionSensor("left_encoder"), *re=robot.getPositionSensor("right_encoder");
  if(!left||!right||!le||!re) {std::cerr<<"Missing wheel devices\n";return 1;}
  webots::Motor *motors[]={left,right}; webots::PositionSensor *enc[]={le,re};
  constexpr int driver_step_ms=10;
  for(int i=0;i<2;i++){enc[i]->enable(driver_step_ms);motors[i]->setPosition(INFINITY);motors[i]->setVelocity(0);}
  auto *rgb=robot.getLED("status_rgb");
  webots::DistanceSensor *reflectance[]={robot.getDistanceSensor("reflectance_left"),robot.getDistanceSensor("reflectance_right")};
  for(auto *sensor:reflectance)if(sensor)sensor->enable(10);
  webots::DistanceSensor *line_reflectance[16];
  for(int i=0;i<16;i++) {
    line_reflectance[i]=robot.getDistanceSensor("line_reflectance_"+std::to_string(i));
    if(line_reflectance[i])line_reflectance[i]->enable(10);
  }
  const int sock=socket(AF_INET,SOCK_DGRAM,0);
  fcntl(sock,F_SETFL,O_NONBLOCK);
  sockaddr_in bind_address{};bind_address.sin_family=AF_INET;
  bind_address.sin_port=htons(port("ROBOTRACERS_COMMAND_PORT",9202));bind_address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  if(bind(sock,reinterpret_cast<sockaddr*>(&bind_address),sizeof(bind_address))) {perror("driver bind");return 1;}
  sockaddr_in destination{};destination.sin_family=AF_INET;
  destination.sin_port=htons(port("ROBOTRACERS_TELEMETRY_PORT",9201));destination.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  sockaddr_in test_destination=destination;
  const int test_port=port("ROBOTRACERS_TEST_TELEMETRY_PORT",0);
  test_destination.sin_port=htons(test_port);
  double last_command=monotonic(), previous_angle[2]={0,0}, angular[2]={0,0}, last_control=0;
  double vision_stamp=0,command_stamp=0,applied_stamp=0,applied_latency=-1;
  std::uint64_t seq=0,last_seq=0; bool have_sequence=false;
  std::string mode="idle";
  // The ODE world still advances at 2 ms. Commands, measured encoders and the
  // zero-order-held DC motor torque update at the local controller's 100 Hz.
  // Avoid 500 IPC round trips/s for a controller that only changes at 100 Hz.
  while(robot.step(driver_step_ms)!=-1) {
    const double now=robot.getTime(), wall=monotonic();
    char packet[2048];ssize_t length;
    while((length=recv(sock,packet,sizeof(packet)-1,0))>0) {
      packet[length]=0;const std::string data(packet);
      const auto incoming=static_cast<std::uint64_t>(number(data,"seq",0));
      const double stamp=number(data,"stamp",wall);
      if(wall-stamp>.25 || stamp-wall>1 || (have_sequence && incoming<=last_seq)) continue;
      mode=word(data,"mode");
      if(mode!="run"&&mode!="idle"&&mode!="pause"&&mode!="stop")continue;
      have_sequence=true;last_seq=incoming;last_command=wall;
      command_stamp=stamp;vision_stamp=number(data,"vision_stamp",0);
      drive.command_v=robotracers::limit(number(data,"v",0),-1.5,1.5);
      drive.command_w=robotracers::limit(number(data,"w",0),-12,12);
      drive.running=mode=="run";
      if(!drive.running)drive.stop();
    }
    if(wall-last_command>.25){drive.stop();mode="idle";have_sequence=false;}
    for(int i=0;i<2;i++) {
      const double a=enc[i]->getValue();
      angular[i]=(a-previous_angle[i])/(driver_step_ms/1000.); previous_angle[i]=a;
      drive.observe(i,a,now);
    }
    if(now-last_control>=.009999) {
      drive.update(now-last_control);last_control=now;
      applied_stamp=monotonic();
      applied_latency=vision_stamp>0 ? std::max(0.,(applied_stamp-vision_stamp)*1000) : -1;
      if(rgb)rgb->set(mode=="run"?((static_cast<int>(now*4)%2)?0x0000ff:0x000008):0x00bb44);
    }
    for(int i=0;i<2;i++)motors[i]->setTorque(drive.torque(drive.wheels[i].pwm,angular[i]));
    if(static_cast<int>(std::round(now/.002))%5==0) {
      std::ostringstream out;out<<std::setprecision(12)
        <<"{\"type\":\"telemetry\",\"seq\":"<<++seq<<",\"sim_time\":"<<now<<",\"stamp\":"<<wall
        <<",\"left_count\":"<<drive.wheels[0].count<<",\"right_count\":"<<drive.wheels[1].count
        <<",\"left_velocity\":"<<drive.wheels[0].velocity<<",\"right_velocity\":"<<drive.wheels[1].velocity
        <<",\"left_pwm\":"<<drive.wheels[0].pwm<<",\"right_pwm\":"<<drive.wheels[1].pwm
        <<",\"left_current\":"<<drive.current(drive.wheels[0].pwm,angular[0])
        <<",\"right_current\":"<<drive.current(drive.wheels[1].pwm,angular[1])
        <<",\"command_seq\":"<<last_seq<<",\"command_stamp\":"<<command_stamp<<",\"applied_stamp\":"<<applied_stamp
        <<",\"command_latency_ms\":"<<(applied_latency>=0?std::to_string(applied_latency):"null")
        <<",\"battery_voltage\":7.4,\"reflectance\":["<<(reflectance[0]?robotracers::limit(reflectance[0]->getValue()/1000,0.,1.):0)
        <<","<<(reflectance[1]?robotracers::limit(reflectance[1]->getValue()/1000,0.,1.):0)<<"],\"line_reflectance\":[";
      for(int i=0;i<16;i++)out<<(i?",":"")<<(line_reflectance[i]?robotracers::limit(line_reflectance[i]->getValue()/1000,0.,1.):0);
      out<<"]}";
      const auto json=out.str();sendto(sock,json.data(),json.size(),0,reinterpret_cast<sockaddr*>(&destination),sizeof(destination));
      if(test_port)sendto(sock,json.data(),json.size(),0,reinterpret_cast<sockaddr*>(&test_destination),sizeof(test_destination));
    }
  }
  close(sock);return 0;
}
