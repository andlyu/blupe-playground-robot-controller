// STS3215 protocol and calibration: docs/refs/feetech/INDEX.md.
// Standalone process: serial polling, interpolation and watchdog never depend on Python.
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif
using Clock = std::chrono::steady_clock;
using Vec = std::array<int, 6>;
static volatile sig_atomic_t stopping = 0;
static void signal_stop(int) { stopping = 1; }
struct Joint { int id, low, high, offset; };
class Bus {
    int fd = -1;
    void send(int id, int instruction, std::vector<unsigned char> params) {
        std::vector<unsigned char> p{255,255,(unsigned char)id,(unsigned char)(params.size()+2),(unsigned char)instruction};
        p.insert(p.end(),params.begin(),params.end());
        unsigned char sum=0; for(size_t i=2;i<p.size();++i) sum+=p[i]; p.push_back(~sum);
        auto until=Clock::now()+std::chrono::milliseconds(30);
        size_t done=0;
        while(done<p.size()) {
            auto n=::write(fd,p.data()+done,p.size()-done);
            if(n>0) done+=n;
            else { if(Clock::now()>until) throw std::runtime_error("serial_write_timeout"); pollfd f{fd,POLLOUT,0}; poll(&f,1,1); }
        }
    }
public:
    explicit Bus(const char* path) {
#ifdef BLUPE_TEST_PTY
        if(std::string(path).rfind("/dev/ttys",0)!=0 && std::string(path).rfind("/dev/pts/",0)!=0) throw std::runtime_error("test_binary_refuses_hardware");
#endif
        fd=open(path,O_RDWR|O_NOCTTY|O_NONBLOCK);
        if(fd<0) throw std::runtime_error("serial_open_failed");
        if(flock(fd,LOCK_EX|LOCK_NB)<0 || ioctl(fd,TIOCEXCL)<0) {close(fd); throw std::runtime_error("serial_busy");}
        termios t{};
        if(tcgetattr(fd,&t)<0) throw std::runtime_error("termios_read_failed");
        cfmakeraw(&t); t.c_cflag|=CLOCAL|CREAD; t.c_cflag&=~(CSTOPB|PARENB|CRTSCTS);
#ifdef __APPLE__
        cfsetispeed(&t,B9600); cfsetospeed(&t,B9600);
#else
        cfsetispeed(&t,B1000000); cfsetospeed(&t,B1000000);
#endif
        if(tcsetattr(fd,TCSANOW,&t)<0) throw std::runtime_error("termios_write_failed");
#if defined(__APPLE__) && !defined(BLUPE_TEST_PTY)
        speed_t speed=1000000;
        if(ioctl(fd,IOSSIOSPEED,&speed)<0) throw std::runtime_error("baud_failed");
#endif
        tcflush(fd,TCIOFLUSH);
    }
    ~Bus() { if(fd>=0) close(fd); }
    int read(int id, int address, int size) {
        tcflush(fd,TCIFLUSH); send(id,2,{(unsigned char)address,(unsigned char)size});
        std::vector<unsigned char> p;
        auto until=Clock::now()+std::chrono::milliseconds(30);
        while(Clock::now()<until) {
            unsigned char b; auto n=::read(fd,&b,1);
            if(n!=1) {pollfd f{fd,POLLIN,0}; poll(&f,1,1); continue;}
            p.push_back(b);
            while(p.size()>=2 && (p[0]!=255 || p[1]!=255)) p.erase(p.begin());
            if(p.size()>=4 && p[3] != size+2) throw std::runtime_error("serial_reply_length");
            if(p.size()==(size_t)(size+6)) {
                unsigned char sum=0; for(size_t i=2;i<p.size();++i) sum+=p[i];
                if(sum!=255 || p[2]!=id || p[4]!=0) throw std::runtime_error("serial_reply_fault");
                return p[5] | (size==2 ? p[6]<<8 : 0);
            }
        }
        throw std::runtime_error("serial_read_timeout_id_"+std::to_string(id)+"_register_"+std::to_string(address));
    }
    void sync(const std::array<Joint,6>& joints,int addr,int size,const Vec& values) {
        std::vector<unsigned char> p{(unsigned char)addr,(unsigned char)size};
        for(int i=0;i<6;++i) {p.push_back(joints[i].id);p.push_back(values[i]&255);if(size==2)p.push_back((values[i]>>8)&255);}
        send(254,131,p);
    }
};
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("usage_port_native_profile");
        std::array<Joint,6> joints{}; std::ifstream cfg(argv[2]);
        for(auto& j:joints) if(!(cfg>>j.id>>j.low>>j.high>>j.offset) || j.id<1 || j.id>252 || j.low<0 || j.high>4095 || j.low>=j.high || (j.offset < -2047 || j.offset > 2047)) throw std::runtime_error("invalid_profile");
        for(int i=0;i<6;++i) for(int k=0;k<i;++k) if(joints[i].id==joints[k].id) throw std::runtime_error("duplicate_id");
        std::string extra;if(cfg>>extra)throw std::runtime_error("invalid_profile");
        signal(SIGPIPE,SIG_IGN);signal(SIGINT,signal_stop);signal(SIGTERM,signal_stop);
        fcntl(STDIN_FILENO,F_SETFL,fcntl(STDIN_FILENO,F_GETFL)|O_NONBLOCK);
        fcntl(STDOUT_FILENO,F_SETFL,fcntl(STDOUT_FILENO,F_GETFL)|O_NONBLOCK);
        Bus bus(argv[1]);
        for(auto j:joints) {
            if(bus.read(j.id,3,2)!=777) throw std::runtime_error("wrong_servo_model");
            int o=bus.read(j.id,31,2); o=(o&2048)?-(o&2047):o;
            if(o!=j.offset) throw std::runtime_error("calibration_mismatch");
            if(bus.read(j.id,33,1)!=0) throw std::runtime_error("not_position_mode");
        }
        Vec pos{}, goal{}, target{}; std::array<double,6> ramp{};
        bool active=false, owns=false, exit_requested=false;
        std::string mode="readonly",error="",input;
        long seq=0; auto heartbeat=Clock::now();auto previous=Clock::now();
        while(!stopping && !exit_requested) {
            auto start=Clock::now(); double dt=std::min(0.05,std::chrono::duration<double>(start-previous).count());previous=start;
            try {
                for(int i=0;i<6;++i) {
                    pos[i]=bus.read(joints[i].id,56,2);
                    if(pos[i]<joints[i].low || pos[i]>joints[i].high) throw std::runtime_error("feedback_outside_limits");
                }
                if(active && Clock::now()-heartbeat>std::chrono::milliseconds(500)) {active=false;mode="hold";error="heartbeat_timeout";}
                if(active) for(int i=0;i<6;++i) if(std::abs(pos[i]-goal[i])>150) throw std::runtime_error("tracking_error");
                char bytes[512]; auto n=::read(STDIN_FILENO,bytes,sizeof(bytes));
                if(n==0) {active=false;exit_requested=true;mode="hold";}
                if(n>0) input.append(bytes,n);
                if(input.size()>2048) throw std::runtime_error("input_too_large");
                // At most eight commands per servo tick; output is never allowed to block.
                for(int count=0;count<8 && input.find('\n')!=std::string::npos;++count) {
                    auto end=input.find('\n');std::istringstream line(input.substr(0,end));input.erase(0,end+1);
                    long request;long long deadline;std::string cmd,tail;
                    if(!(line>>request>>deadline>>cmd) || request<=seq) throw std::runtime_error("invalid_command");
                    seq=request;
                    auto wall=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                    if(cmd!="hold" && cmd!="close" && (deadline<wall || deadline>wall+1000)) throw std::runtime_error("stale_command");
                    if(cmd=="target") {
                        Vec next;for(auto& v:next) if(!(line>>v)) throw std::runtime_error("invalid_target");
                        if(line>>tail) throw std::runtime_error("invalid_target");
                        if(!active) throw std::runtime_error("not_enabled");
                        for(int i=0;i<6;++i) if(next[i]<joints[i].low || next[i]>joints[i].high) throw std::runtime_error("target_outside_limits");
                        for(int i=0;i<6;++i) if(std::abs(next[i]-pos[i])>228) throw std::runtime_error("target_delta_limit");
                        target=next;heartbeat=start;
                    } else {
                        if(line>>tail)throw std::runtime_error("invalid_command");
                        if(cmd=="ping") heartbeat=start;
                        else if(cmd=="enable") {
                            if(mode=="fault")throw std::runtime_error("restart_required");
                            goal=target=pos;for(int i=0;i<6;++i)ramp[i]=pos[i];
                            bus.sync(joints,42,2,goal); owns=true;
                            bus.sync(joints,40,1,Vec{1,1,1,1,1,1});
                            active=true;mode="active";error="";heartbeat=start;
                        } else if(cmd=="hold" || cmd=="close") {
                            active=false;mode=owns?"hold":"readonly";exit_requested=cmd=="close";
                        } else throw std::runtime_error("unknown_command");
                    }
                }
                if(active) for(int i=0;i<6;++i) {ramp[i]+=std::clamp(target[i]-ramp[i],-100*dt,100*dt);goal[i]=(int)std::round(ramp[i]);}
                if(owns) bus.sync(joints,42,2,goal);
            } catch(const std::exception& e) {
                active=false;mode="fault";error=e.what();
                // Retain the last bounded setpoint; never torque off an unsupported arm.
                if(owns)try{bus.sync(joints,42,2,goal);}catch(...){}
            }
            std::ostringstream out;out<<"{\"seq\":"<<seq<<",\"mode\":\""<<mode<<"\",\"error\":\""<<error<<"\",\"raw\":[";
            for(int i=0;i<6;++i)out<<(i?",":"")<<pos[i];out<<"]}\n";
            auto text=out.str();(void)::write(STDOUT_FILENO,text.data(),text.size());
            std::this_thread::sleep_until(start+std::chrono::milliseconds(20));
        }
        // Last setpoint remains in the servo on clean shutdown or parent EOF.
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<"\n";return 1;}
}
