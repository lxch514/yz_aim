#include "shared_can_bus.h"
#include "frame_trace.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fcntl.h>
#include <unistd.h>

using namespace yz_aim;
namespace {
int checks=0;
void require(bool ok,const char* reason) {
    ++checks;
    if(!ok) { std::fprintf(stderr,"FAIL %s\n",reason); std::exit(1); }
}
class Bus : public Transport {
public:
    std::deque<Frame> queue;
    unsigned sends=0;
    bool send(const Frame&) override { ++sends;return true; }
    bool receive(Frame& frame,Deadline until) override {
        if(Clock::now()>=until || queue.empty()) return false;
        frame=queue.front();queue.pop_front();return true;
    }
};
}
int main() {
    Bus physical;
    SharedCanBus router(physical,1,2);
    Frame frame;
    physical.queue.push_back({0x582,8,{0x60,0x40,0x60,0}});
    physical.queue.push_back({0x581,8,{0x60,0x40,0x60,0}});
    require(router.first().receive(frame,Clock::now()+std::chrono::milliseconds(20)) && frame.id==0x581,
        "A sees own ACK even when B replies first");
    require(router.second().receive(frame,Clock::now()+std::chrono::milliseconds(20)) && frame.id==0x582,
        "B ACK preserved in logical queue");
    for(int i=0;i<10000;++i) physical.queue.push_back({0x182,8,{static_cast<uint8_t>(i)}});
    physical.queue.push_back({0x582,8,{0x60,0xff,0x60,0}});
    physical.queue.push_back({0x581,8,{0x60,0xff,0x60,0}});
    require(router.first().receive(frame,Clock::now()+std::chrono::seconds(1)) && frame.id==0x581,
        "other-axis PDO burst does not hide A SDO");
    require(router.second().receive(frame,Clock::now()+std::chrono::milliseconds(20)) && frame.id==0x582,
        "other-axis SDO survives telemetry coalescing");
    require(router.telemetry_coalesced()==9999 && !router.first().failed(),
        "bounded telemetry memory without dropping SDO ACK");
    require(!router.first().accept_sdo_replies(1,2),"dual node-ID collision rejected");
    SharedCanBus config(physical,1,0);
    require(config.first().accept_sdo_replies(1,2),"single-port ID migration accepts new reply ID");
    physical.queue.push_back({0x582,8,{0x60,0x15,0x26,0}});
    require(config.first().receive(frame,Clock::now()+std::chrono::milliseconds(20)) && frame.id==0x582,
        "new node reply routed during migration");
    require(config.first().accept_sdo_replies(1,0),"ID migration filter restored");

    int fds[2]; require(::pipe2(fds,O_NONBLOCK|O_CLOEXEC)==0,"diagnostic output pipe");
    require(fcntl(fds[1],F_GETFL)>=0,"pipe flags available");
    // Fill the pipe so the diagnostic writer cannot make any progress.
    char buffer[4096]{}; while(::write(fds[1],buffer,sizeof(buffer))>0) {}
    require(fcntl(fds[1],F_SETFL,fcntl(fds[1],F_GETFL)&~O_NONBLOCK)==0,"original application stderr is blocking");
    const int old_flags=fcntl(fds[1],F_GETFL);
    const auto started=Clock::now();
    {
        FrameTrace trace("congested-test",true,fds[1]);
        for(unsigned i=0;i<100000;++i) trace.frame("TX",Frame{0x601,8,{0x40,0x64,0x60}});
        trace.frame("RX",Frame{0x581,8,{0x43,0x64,0x60}});
        trace.frame("TX_ERR",Frame{0x602,8,{0x40,0x64,0x60}});
        trace.kernel_dropped(7);
        const auto counts=trace.counts();
        require(counts.tx==100000 && counts.rx==1 && counts.tx_error==1 && counts.kernel_drop==7,
            "transport counters independent from diagnostic output losses");
        require(counts.trace_drop+counts.output_drop>0,"congested diagnostics counted explicitly");
    }
    require(Clock::now()-started<std::chrono::seconds(3),"full terminal cannot block CAN producer or shutdown");
    require(fcntl(fds[1],F_GETFL)==old_flags,"diagnostics do not change application output flags");
    ::close(fds[0]);::close(fds[1]);
    int closed[2];require(::pipe2(closed,O_NONBLOCK|O_CLOEXEC)==0,"closed output test pipe");
    ::close(closed[0]);
    { FrameTrace trace("closed-test",true,closed[1]); trace.frame("TX",Frame{0x601,8,{}}); }
    ::close(closed[1]);
    require(true,"closed terminal pipe does not terminate control process with SIGPIPE");
    std::printf("diagnostic_test: %d checks passed (routing and trace backpressure)\n",checks);
}
