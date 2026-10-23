// Offline only: include the real CLI/control loop, drive it through an in-memory bus.
// The SocketCanTransport below is never opened; no physical CAN interface is used.
#define main primitive_test_main
#include "yz_aim_sync_test.cpp"
#undef main
#define main application_main
#include "../src/yz_aim_can0.cpp"
#undef main

#include <fstream>
#include <fcntl.h>
#include <set>

namespace {
struct SentAt { yz_aim::Frame frame; yz_aim::Clock::time_point at; };
class RuntimeBus : public DualBus {
public:
    std::vector<SentAt> timeline;
    std::chrono::milliseconds read_delay{0};
    std::chrono::milliseconds target_ack_delay{0};
    unsigned drop_start_ack_mask=0, ignore_start_mask=0, wrong_target_mask=0;
    std::chrono::milliseconds start_ack_delay{0};
    bool late_start_abort=false;
    unsigned delayed_node = 0;
    uint16_t delayed_object = 0;
    bool delay_only_while_running = false;
    bool running[3] = {};
    double measured_position[3] = {};
    bool integrate_motion = false;
    bool inject_step = false, step_done = false;
    yz_aim::Clock::time_point first_start{};
    yz_aim::Clock::time_point updated = yz_aim::Clock::now();
    std::deque<std::pair<yz_aim::Clock::time_point,yz_aim::Frame>> delayed;
    void integrate() {
        const auto now=yz_aim::Clock::now();
        const double dt=std::chrono::duration<double>(now-updated).count();
        updated=now;
        if (!integrate_motion) return;
        for(unsigned id : {1U,2U}) {
            const int32_t target=static_cast<int32_t>(node[id].registers[0x60ff]);
            const double speed=running[id] ? target*(id==1 ? 0.998 : 0.9943333333) : 0;
            measured_position[id]+=speed*yz_aim::kMotorPulsesPerRevolution/60.0*dt;
            node[id].position=static_cast<int32_t>(std::llround(measured_position[id]));
            node[id].speed=static_cast<int16_t>(std::lround(speed*10));
        }
        if (inject_step && !step_done && first_start!=yz_aim::Clock::time_point{} &&
            now-first_start>std::chrono::milliseconds(600)) {
            measured_position[2]+=6000;
            node[2].position=static_cast<int32_t>(std::llround(measured_position[2]));
            step_done=true;
        }
    }
    bool send(const yz_aim::Frame& f) override {
        integrate();
        timeline.push_back({f,yz_aim::Clock::now()});
        const auto previous=replies.size();
        const bool ok=DualBus::send(f);
        if(f.id!=0x601 && f.id!=0x602) return ok;
        const unsigned id=f.id-0x600;
        const auto object=index_of(f);
        if(f.data[0]!=0x40 && object==0x6040) {
            const auto value=value_of(f);
            if(value&0x100) { running[id]=false; node[id].speed=0; }
            else if((value&0xf)==0xf) {
                running[id]=true;
                if(first_start==yz_aim::Clock::time_point{} && static_cast<int32_t>(node[id].registers[0x60ff])!=0)
                    first_start=yz_aim::Clock::now();
            }
            if(value==0x100) node[id].output=6;
            if(value==0xf && static_cast<int32_t>(node[id].registers[0x60ff])!=0) {
                if(ignore_start_mask&(1U<<id))running[id]=false;
                else node[id].output=5;
                if(wrong_target_mask&(1U<<id))node[id].registers[0x60ff]^=0x80000000U;
                if(replies.size()>previous && ((drop_start_ack_mask&(1U<<id)) || start_ack_delay.count())) {
                    auto reply=replies.back();replies.pop_back();
                    if(start_ack_delay.count()) {
                        if(late_start_abort) {reply.data[0]=0x80;reply.data[4]=0x30;reply.data[5]=0;reply.data[6]=9;reply.data[7]=6;}
                        delayed.push_back({yz_aim::Clock::now()+start_ack_delay,reply});
                    }
                }
            }
        }
        if(f.data[0]==0x40 && read_delay.count() && id==delayed_node &&
            (!delayed_object || object==delayed_object) &&
            (!delay_only_while_running || (running[id] && static_cast<int32_t>(node[id].registers[0x60ff])!=0)) && replies.size()>previous) {
            auto reply=replies.back(); replies.pop_back();
            delayed.push_back({yz_aim::Clock::now()+read_delay,reply});
        }
        if(f.data[0]!=0x40 && object==0x60ff && static_cast<int32_t>(value_of(f))!=0 &&
            id==1 && target_ack_delay.count() && replies.size()>previous) {
            auto reply=replies.back(); replies.pop_back();
            delayed.push_back({yz_aim::Clock::now()+target_ack_delay,reply});
        }
        return ok;
    }
    bool receive(yz_aim::Frame& f,yz_aim::Deadline until) override {
        for(;;) {
            const auto now=yz_aim::Clock::now();
            while(!delayed.empty() && delayed.front().first<=now) {
                replies.push_back(delayed.front().second); delayed.pop_front();
            }
            if(!replies.empty()) return DualBus::receive(f,until);
            if(now>=until) return false;
            auto next=until;
            if(!delayed.empty()) next=std::min(next,delayed.front().first);
            std::this_thread::sleep_until(next);
        }
    }
};

// Model the production sockets' independent receive queues, including B replying before A.
struct PortRouter {
    RuntimeBus& bus;
    std::array<std::deque<yz_aim::Frame>,3> queue;
    bool receive(unsigned node,yz_aim::Frame& frame,yz_aim::Deadline deadline) {
        for(;;) {
            if(!queue[node].empty()) { frame=queue[node].front();queue[node].pop_front();return true; }
            yz_aim::Frame incoming;
            if(!bus.receive(incoming,deadline)) return false;
            if(incoming.id==0x581 || incoming.id==0x582) queue[incoming.id-0x580].push_back(incoming);
        }
    }
};
struct NodePort : yz_aim::Transport {
    PortRouter& router; unsigned node;
    NodePort(PortRouter& r,unsigned n):router(r),node(n){}
    bool send(const yz_aim::Frame& f) override { return router.bus.send(f); }
    bool receive(yz_aim::Frame& f,yz_aim::Deadline d) override { return router.receive(node,f,d); }
};

class NodeChangeBus : public RuntimeBus {
public:
    uint8_t reply_node=1;
    uint16_t saved_node=1;
    std::vector<uint16_t> allowed=yz_aim::SocketCanTransport::receive_ids(1);
    bool installed_before_write=false, restored=false;
    yz_aim::Clock::time_point flash_started{};
    unsigned save_polls=0;
    bool accept_sdo_replies(uint8_t primary,uint8_t alternate) override {
        allowed=yz_aim::SocketCanTransport::receive_ids(primary,alternate);
        if(alternate==2 && saved_node==1) installed_before_write=true;
        if(!alternate && saved_node==2) restored=true;
        return true;
    }
    bool send(const yz_aim::Frame& f) override {
        if(f.id!=0x601 || (index_of(f)!=0x2615 && index_of(f)!=0x2614)) return RuntimeBus::send(f);
        timeline.push_back({f,yz_aim::Clock::now()}); sent.push_back(f);
        yz_aim::Frame reply{0,8,{0,f.data[1],f.data[2],f.data[3],0,0,0,0}};
        if(f.data[0]==0x40) {
            reply.data[0]=0x4b;
            if(index_of(f)==0x2615) reply.data[4]=saved_node;
            else {
                ++save_polls;
                reply.data[4]=yz_aim::Clock::now()-flash_started>=std::chrono::milliseconds(450) ? 2 : 1;
            }
        } else {
            reply.data[0]=0x60;
            if(index_of(f)==0x2615) { saved_node=value_of(f); reply_node=saved_node; }
            else flash_started=yz_aim::Clock::now();
        }
        reply.id=0x580+reply_node;
        replies.push_back(reply);
        return true;
    }
    bool receive(yz_aim::Frame& f,yz_aim::Deadline until) override {
        while(RuntimeBus::receive(f,until))
            if(std::find(allowed.begin(),allowed.end(),f.id)!=allowed.end()) return true;
        return false;
    }
};

void test_command_node_change() {
    NodeChangeBus bus;
    bus.node[1].output=5; // observed enabled state; the CLI must stop and disable first
    yz_aim::Motor motor(bus,1);
    yz_aim::SocketCanTransport unopened("can0",1);
    Session session{motor,unopened,1,3000,true};
    const auto disposition=command_set_node(session,"2");
    require(disposition==Disposition::quit && session.command_rc==0,"set-node completes and terminates transition session");
    require(bus.installed_before_write && bus.restored,"kernel reply IDs installed before address write and restored after");
    require(std::find(bus.allowed.begin(),bus.allowed.end(),0x582)==bus.allowed.end(),"new reply ID is scoped to the configuration transaction");
    require(bus.saved_node==2 && bus.reply_node==2 && bus.save_polls>=5,"new-ID ACK, readback and delayed flash completion accepted");
    require(!yz_aim::output_is_enabled(bus.node[1].output),"set-node leaves output disabled");
    unsigned writes=0;
    for(const auto& f:bus.sent) if(f.id==0x601 && f.data[0]!=0x40 && index_of(f)==0x2615) ++writes;
    require(writes==1,"address write is never blindly retried");
    auto ids=yz_aim::SocketCanTransport::receive_ids(1,2);
    require(std::find(ids.begin(),ids.end(),0x581)!=ids.end() && std::find(ids.begin(),ids.end(),0x582)!=ids.end(),
            "actual SocketCAN filter ID factory includes old and new SDO replies");
    require(std::find(ids.begin(),ids.end(),0x583)==ids.end(),"unrelated node not admitted");
    NodeChangeBus denied_bus;
    yz_aim::Motor denied_motor(denied_bus,1);
    Session denied{denied_motor,unopened,1,3000,false};
    require(run_single_command(denied,{"set-node","2"})==2,"one-shot configuration refusal returns nonzero exit status");
    require(denied_bus.sent.empty(),"configuration without --yes never sends a CAN request");
}

void test_shared_heartbeat_during_slow_other_axis() {
    RuntimeBus bus;
    auto heartbeat=std::make_shared<yz_aim::SharedHeartbeat>();
    yz_aim::Motor a(bus,1,heartbeat),b(bus,2,heartbeat);
    std::string why;
    require(bring_up(bus,a,1,why),"shared A enabled: "+why);
    require(bring_up(bus,b,2,why),"shared B enabled: "+why);
    bus.timeline.clear();
    bus.delayed_node=2; bus.read_delay=std::chrono::milliseconds(80);
    const auto started=yz_aim::Clock::now();
    auto sample=b.snapshot(after(std::chrono::milliseconds(4000)));
    require(static_cast<bool>(sample.result),"slow B snapshot remains readable: "+sample.result.message);
    require(yz_aim::Clock::now()-started>std::chrono::milliseconds(700),"B occupies the worker longer than old A-local watchdog limit");
    require(static_cast<bool>(a.service_watchdog(after())),"A does not falsely report overdue after B serviced shared producer");
    auto last=started; unsigned beats=0;
    for(const auto& item:bus.timeline) if(item.frame.id==0x77f) {
        require(item.at-last<std::chrono::milliseconds(300),"shared producer serviced during SDO waits");
        last=item.at; ++beats;
    }
    require(beats>=3,"multiple heartbeats sent while only B was being sampled");
    heartbeat->last_sent=yz_aim::Clock::now()-std::chrono::milliseconds(750);
    const auto count=bus.timeline.size();
    require(!b.service_watchdog(after()) && heartbeat->faulted,"genuinely overdue producer latches group fault");
    require(!a.service_watchdog(after()),"other axis observes shared fault");
    for(size_t i=count;i<bus.timeline.size();++i)
        require(bus.timeline[i].frame.id!=0x77f,"fault does not resume heartbeat and stale motion");
}

void test_sdo_request_spacing() {
    RuntimeBus bus;yz_aim::SdoClient client(bus,2);uint32_t value=0;
    require(static_cast<bool>(client.read(0x6041,0,2,value,after())),"first paced SDO read succeeds");
    const auto first=yz_aim::Clock::now();
    require(static_cast<bool>(client.read(0x260e,0,2,value,after())),"second paced SDO read succeeds");
    require(bus.timeline.back().at-first>=std::chrono::microseconds(9800),"same-node request respects reply recovery gap");
    const auto count=bus.timeline.size();
    client.set_hard_deadline(yz_aim::Clock::now()+std::chrono::microseconds(200));
    require(!client.read(0x6041,0,2,value,after()),"hard motion deadline expires during pacing before transmit");
    require(bus.timeline.size()==count,"no request emitted after pacing deadline");
}

void test_50_second_controller_models() {
    for(int direction:{1,-1}) {
        yz_aim::VelocitySynchronizer controller;
        controller.reset(1000000000,-1000000000,-1,direction*300,5);
        double pa=1000000000,pb=-1000000000;
        int ca=-direction*300,cb=direction*300;
        double maximum=0;
        for(int i=0;i<500;++i) {
            pa+=ca*0.998*yz_aim::kMotorPulsesPerRevolution/60.0*0.1;
            pb+=cb*0.9943333333*yz_aim::kMotorPulsesPerRevolution/60.0*0.1;
            yz_aim::SyncFeedback f{true,std::llround(pa),std::llround(pb),0};
            auto trim=controller.update(f,0.1,i>=8);
            ca=trim.rpm_a;cb=trim.rpm_b;
            maximum=std::max(maximum,std::abs(trim.error_deg));
            require(ca*direction<0 && cb*direction>0 && std::abs(ca)<=300 && std::abs(cb)<=300,
                    "50s model keeps mirror direction and command speed bound");
            require(std::abs(ca)>=270 && std::abs(cb)>=270,"trim cannot exceed configured nominal percentage bound");
        }
        require(maximum<0.5,"50s model corrects persistent 1.1RPM mismatch without exceeding unchanged conservative guard");
        std::printf("50-second model %s max_error=%.6f output degrees\n",direction>0?"extend":"retract",maximum);
    }
    yz_aim::VelocitySynchronizer controller;
    controller.reset(0,0,-1,300,5);
    yz_aim::SyncFeedback step{true,0,6000,0};
    for(int i=0;i<5;++i)
        require(controller.update(step,0.1,true).error_pulses==6000,"permanent offset is never rebased away");
    auto now=yz_aim::Clock::now();
    yz_aim::Snapshot a{},b{};
    a.position_sample=b.position_sample=a.speed_sample=b.speed_sample={true,now};
    require(yz_aim::aligned_feedback(a,b,now).usable,"fresh paired samples accepted");
    a.position_sample.retried=true;
    require(!yz_aim::aligned_feedback(a,b,now).usable,"retried position not silently used for feedback correction");
    a.position_sample={true,now-std::chrono::seconds(1)};
    require(!yz_aim::aligned_feedback(a,b,now).usable,"stale feedback rejected");
    // Accelerating mirrors sampled 120ms apart: velocity-only extrapolation creates false skew.
    const auto origin=now-std::chrono::milliseconds(360);
    const double k=yz_aim::kMotorPulsesPerRevolution/60.0;
    a={};b={};
    a.position_sample={true,origin+std::chrono::milliseconds(230)};
    b.position_sample={true,origin+std::chrono::milliseconds(350)};
    a.speed_sample={true,origin+std::chrono::milliseconds(200)};
    b.speed_sample={true,origin+std::chrono::milliseconds(320)};
    a.speed_rpm=-100;b.speed_rpm=160;
    a.position_pulses=std::lround(-0.5*500*k*0.23*0.23);
    b.position_pulses=std::lround(0.5*500*k*0.35*0.35);
    const auto aligned=yz_aim::aligned_feedback(a,b,now,-500,500,300);
    require(aligned.usable && std::abs(aligned.position_a+aligned.position_b)<=2,
            "timestamped acceleration alignment removes startup sampling skew without loosening hard guard");
}

void test_coordinated_cancel_does_not_stop_one_axis_first() {
    RuntimeBus bus;
    bus.integrate_motion=true;
    auto heartbeat=std::make_shared<yz_aim::SharedHeartbeat>();
    yz_aim::Motor a(bus,1,heartbeat);
    yz_aim::SocketCanTransport unopened("can0",1);
    Session session{a,unopened,1,3000,true};
    session.second=std::make_unique<yz_aim::Motor>(bus,2,heartbeat);
    session.node2=2;
    std::string why;
    require(bring_up(bus,a,1,why) && bring_up(bus,*session.second,2,why),"cancel test enables both fake drives");
    require(static_cast<bool>(a.stage_velocity(-30,std::chrono::seconds(1),after())) &&
            static_cast<bool>(session.second->stage_velocity(30,std::chrono::seconds(1),after())),"cancel test stages both axes");
    a.set_motion_deadline(after());session.second->set_motion_deadline(after());
    require(static_cast<bool>(a.trigger_velocity(after())) && static_cast<bool>(session.second->trigger_velocity(after())),"cancel test triggers both axes");
    const auto count=bus.sent.size();
    a.request_cancel();
    const auto r=a.update_velocity(-29,after());
    yz_aim::Snapshot sample;
    const auto rb=session.second->poll_safety(sample,after());
    require(r.error==yz_aim::Error::cancelled && rb.error==yz_aim::Error::cancelled,"cancel is seen by both axes");
    require(bus.sent.size()==count,"coordinated cancellation never enters a blocking single-axis stop first");
    require(!a.unsafe_latched() && !session.second->unsafe_latched(),"cancel before write is not a drive fault");
    require(sync_halt_both(session),"coordinator stops and confirms both axes");
    require(!heartbeat->cancel_requested.load(),"confirmed group stop clears shared cancellation");
}

void test_paired_start_does_not_wait_for_first_axis_ack(unsigned delay_ms=70) {
    RuntimeBus bus;
    bus.integrate_motion=true;
    PortRouter router{bus,{}};
    NodePort port_a(router,1),port_b(router,2);
    auto heartbeat=std::make_shared<yz_aim::SharedHeartbeat>();
    yz_aim::Motor a(port_a,1,heartbeat);
    yz_aim::SocketCanTransport unopened("can0",1);
    Session session{a,unopened,1,3000,true};
    session.second=std::make_unique<yz_aim::Motor>(port_b,2,heartbeat);
    session.node2=2;
    std::string why;
    require(bring_up(bus,a,1,why) && bring_up(bus,*session.second,2,why),"paired start enables both fake drives");
    require(static_cast<bool>(a.stage_velocity(-30,std::chrono::seconds(1),after())) &&
            static_cast<bool>(session.second->stage_velocity(30,std::chrono::seconds(1),after())),"paired start stages both axes");
    a.set_motion_deadline(after());session.second->set_motion_deadline(after());
    bus.target_ack_delay=std::chrono::milliseconds(delay_ms);
    const auto begin=bus.timeline.size();
    require(static_cast<bool>(a.trigger_velocity_pair(*session.second,after())),"paired trigger handles B ACK arriving before delayed A ACK");
    std::vector<yz_aim::Clock::time_point> targets,starts;
    for(size_t i=begin;i<bus.timeline.size();++i) {
        const auto& item=bus.timeline[i];
        if(item.frame.id!=0x601 && item.frame.id!=0x602)continue;
        if(index_of(item.frame)==0x60ff && item.frame.data[0]==0x23) targets.push_back(item.at);
        if(index_of(item.frame)==0x6040 && value_of(item.frame)==0xf) starts.push_back(item.at);
    }
    require(targets.size()==2 && starts.size()==2,"paired start sends each target and controlword exactly once");
    require(targets[1]-targets[0]<std::chrono::milliseconds(15),"target frames are not separated by A's 70ms ACK delay");
    require(starts[1]-starts[0]<std::chrono::milliseconds(15),"start controlwords are also sent before either ACK is awaited");
    require(sync_halt_both(session),"paired start test confirms both outputs stopped");
}

void test_real_cli_loop(bool delayed_read, bool dangerous_step, bool equal_speed=false) {
    RuntimeBus bus;
    bus.integrate_motion=true;
    bus.inject_step=dangerous_step;
    auto heartbeat=std::make_shared<yz_aim::SharedHeartbeat>();
    yz_aim::Motor a(bus,1,heartbeat);
    yz_aim::SocketCanTransport unopened("can0",1);
    Session session{a,unopened,1,3000,true};
    session.second=std::make_unique<yz_aim::Motor>(bus,2,heartbeat);
    session.node2=2;
    session.log_directory=motion_log_directory().parent_path()/"test/runtime-logs";
    if(equal_speed) {
        command_sync_mode(session,"speed");
        require(session.equal_speed_test,"explicit speed mode selected");
        command_sync_mode(session,"invalid");
        require(session.equal_speed_test,"invalid mode cannot change active selection");
        require(session.sync_tolerance_deg==0.5 && session.correction_rpm==5,
                "speed mode preserves position-mode protection settings");
    }
    command_sync_enable(session);
    require(session.dual_enabled,"real CLI enables both fake drives");
    const auto count=bus.sent.size();
    command_sync_extend(session,{"30","150"},true);
    require(bus.sent.size()==count,"motion without explicit initial zero rejected without any CAN write");
    command_sync_zero(session);
    require(session.sync_zero_valid,"real CLI records fixed initial zero");
    const auto za=session.sync_zero_a,zb=session.sync_zero_b;
    if(delayed_read) {
        bus.delayed_node=1;bus.delayed_object=0x6064;
        bus.read_delay=std::chrono::milliseconds(600);
        bus.delay_only_while_running=true;
    }
    const unsigned duration=dangerous_step?1200:delayed_read?120:650;
    command_sync_extend(session,{"30",std::to_string(duration)},true);
    require(!session.motion_log && session.stopped_at!=yz_aim::Clock::time_point{},"both stops confirmed and CSV finalized");
    require(session.sync_zero_a==za && session.sync_zero_b==zb,"action never changes fixed zero");
    const auto actual=std::chrono::duration_cast<std::chrono::milliseconds>(session.stop_a_at-session.sync_move_started).count();
    if(dangerous_step && !equal_speed) {
        require(bus.step_done && session.end_reason.find("fault:")==0,"permanent dangerous skew still stops the real CLI loop");
        require(actual<duration,"dangerous skew stops before requested duration");
    } else {
        require(session.end_reason=="requested duration elapsed","time boundary is normal completion, not fault");
        require(actual>=duration && actual<duration+60,"SDO wait cannot postpone timed stop by hundreds of milliseconds");
        require(!a.unsafe_latched() && !session.second->unsafe_latched(),"normal end does not poison either motor session");
        if(equal_speed && dangerous_step) {
            require(bus.step_done && session.max_error_deg>0.5,"speed test records position mismatch above old guard");
            for(const auto& f:bus.sent) if((f.id==0x601 || f.id==0x602) && f.data[0]!=0x40 && index_of(f)==0x60ff) {
                const auto v=static_cast<int32_t>(value_of(f));
                require(v==0 || v==(f.id==0x601?-30:30),"position mismatch cannot alter equal mirrored speed targets");
            }
        }
    }
    require(session.stop_b_at-session.stop_a_at<std::chrono::milliseconds(20),"stop frames go to both axes before waiting for acknowledgements");
    bus.read_delay=std::chrono::milliseconds(0);
    if(!dangerous_step || equal_speed) {
        session.sync_run_active=true;
        if(equal_speed) {
            command_sync_mode(session,"position");
            require(session.equal_speed_test,"active motion refuses mode change");
        }
        command_sync_zero(session);
        require(session.sync_zero_a==za && session.sync_zero_b==zb,"active motion cannot reset zero");
        session.sync_run_active=false;
        if(!delayed_read) {
            command_sync_extend(session,{"30","650"},false);
            require(session.end_reason=="requested duration elapsed","retract follows extend in the same session");
            require(session.sync_zero_a==za && session.sync_zero_b==zb,"extend/retract sequence keeps one fixed zero");
            require(session.target_rpm_a>0 && session.target_rpm_b<0,"retract reverses both mirror signs");
        }
    }
    command_sync_disable(session);
    require(!yz_aim::output_is_enabled(bus.node[1].output) && !yz_aim::output_is_enabled(bus.node[2].output),
            "CLI cleanup disables both fake outputs");
    if(equal_speed) {
        command_sync_mode(session,"position");
        require(!session.equal_speed_test && session.sync_tolerance_deg==0.5,
                "position mode restores unchanged protection settings");
    }
}

void test_missing_start_ack(unsigned mask, bool ignore_start=false, bool wrong_target=false,
                            bool delayed_abort=false, bool delayed_ack=false) {
    RuntimeBus bus;bus.integrate_motion=true;PortRouter router{bus,{}};
    NodePort pa(router,1),pb(router,2);auto beat=std::make_shared<yz_aim::SharedHeartbeat>();
    yz_aim::Motor a(pa,1,beat);Session session{a,pa,1,3000,true};
    session.second=std::make_unique<yz_aim::Motor>(pb,2,beat);session.node2=2;
    std::string why;require(bring_up(bus,a,1,why) && bring_up(bus,*session.second,2,why),"ACK loss fixture enabled");
    require(static_cast<bool>(a.stage_velocity(-300,std::chrono::seconds(3),after())) &&
            static_cast<bool>(session.second->stage_velocity(300,std::chrono::seconds(3),after())),"ACK loss fixture staged");
    bus.drop_start_ack_mask=mask;
    if(ignore_start)bus.ignore_start_mask=mask;
    if(wrong_target)bus.wrong_target_mask=mask;
    if(delayed_abort || delayed_ack) {bus.start_ack_delay=std::chrono::milliseconds(350);bus.late_start_abort=delayed_abort;}
    const auto until=after(std::chrono::milliseconds(1600));a.set_motion_deadline(until);session.second->set_motion_deadline(until);
    const auto first=bus.timeline.size();const auto result=a.trigger_velocity_pair(*session.second,until);
    const bool success=!(ignore_start || wrong_target || delayed_abort);
    require(static_cast<bool>(result)==success,"missing START ACK accepted only with independently verified running state: "+result.message);
    unsigned starts=0,targets=0;
    for(size_t i=first;i<bus.timeline.size();++i) {
        const auto& frame=bus.timeline[i].frame;
        if(frame.id!=0x601 && frame.id!=0x602)continue;
        if(frame.data[0]!=0x40 && index_of(frame)==0x6040 && value_of(frame)==0xf)++starts;
        if(frame.data[0]!=0x40 && index_of(frame)==0x60ff && static_cast<int32_t>(value_of(frame))!=0)++targets;
    }
    require(starts==2 && targets==2,"lost ACK never replays target or START write");
    if(success) {
        require(result.context=="paired_start_confirmed_by_readback","readback confirmation is explicitly identified");
        require(!a.unsafe_latched() && !session.second->unsafe_latched() && !beat->faulted,"verified lost ACK does not poison either session");
    } else {
        require(a.unsafe_latched() && session.second->unsafe_latched(),"unproved start locks both axes");
        require(!a.arm(after()) && !a.update_velocity(-299,after()),"faulted session cannot resume motion");
    }
    const auto cleanup_begin=bus.timeline.size();
    require(sync_halt_both(session),"stop can be freshly confirmed even after ambiguous START write");
    require(static_cast<bool>(a.disable_output(after())) && static_cast<bool>(session.second->disable_output(after())),"both output disables confirmed after missing ACK");
    if(!success)for(size_t i=cleanup_begin;i<bus.timeline.size();++i) {
        const auto& f=bus.timeline[i].frame;
        require(f.id!=0x77f,"fault cleanup never resumes producer heartbeat");
        if((f.id==0x601 || f.id==0x602) && f.data[0]!=0x40 && index_of(f)==0x6040)
            require(value_of(f)==0x100,"fault cleanup only writes operation-clear controlword");
    }
}

void test_cli_repeat_after_missing_start_ack() {
    RuntimeBus bus;bus.integrate_motion=true;PortRouter router{bus,{}};NodePort pa(router,1),pb(router,2);
    auto beat=std::make_shared<yz_aim::SharedHeartbeat>();yz_aim::Motor a(pa,1,beat);Session session{a,pa,1,3000,true};
    session.second=std::make_unique<yz_aim::Motor>(pb,2,beat);session.node2=2;
    session.log_directory=motion_log_directory().parent_path()/"test/runtime-logs";
    command_sync_mode(session,"speed");command_sync_enable(session);command_sync_zero(session);
    bus.drop_start_ack_mask=1U<<1;
    for(bool extend:{true,false,true,false}) {
        command_sync_extend(session,{"300","1200"},extend);
        const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(session.stop_a_at-session.sync_move_started).count();
        require(session.end_reason=="requested duration elapsed" && ms>=1200 && ms<1260,"repeated missing ACK still honors original motion deadline");
        require(!a.unsafe_latched() && !session.second->unsafe_latched(),"following commands remain usable after verified START");
    }
    command_sync_disable(session);
}

void test_45_second_cli_with_trace(bool equal_speed_minute=false) {
    const auto trace_file=motion_log_directory().parent_path()/(equal_speed_minute ? "test/equal-speed-fake-can.log" : "test/long-can-trace.log");
    const int fd=::open(trace_file.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC,0600);
    require(fd>=0,"long test diagnostic log opened inside project");
    yz_aim::FrameTrace trace("long-fake-bus",true,fd);
    class TracedBus final : public RuntimeBus {
    public:
        yz_aim::FrameTrace& trace;
        explicit TracedBus(yz_aim::FrameTrace& t):trace(t){}
        bool send(const yz_aim::Frame& frame) override {
            const bool ok=RuntimeBus::send(frame);trace.frame(ok?"TX":"TX_ERR",frame);return ok;
        }
        bool receive(yz_aim::Frame& frame,yz_aim::Deadline until) override {
            const bool ok=RuntimeBus::receive(frame,until);if(ok)trace.frame("RX",frame);return ok;
        }
    } bus(trace);
    bus.integrate_motion=true;
    bus.inject_step=equal_speed_minute;
    PortRouter router{bus};
    NodePort port_a(router,1),port_b(router,2);
    auto heartbeat=std::make_shared<yz_aim::SharedHeartbeat>();
    yz_aim::Motor a(port_a,1,heartbeat);
    Session session{a,port_a,1,3000,true};
    session.second=std::make_unique<yz_aim::Motor>(port_b,2,heartbeat);
    session.node2=2;
    session.log_directory=motion_log_directory().parent_path()/(equal_speed_minute ? "test/equal-speed-minute-logs" : "test/runtime-logs");
    if(equal_speed_minute) {
        std::printf("OFFLINE SIMULATION: real CLI, fake drives only; +/-300RPM, 3 seconds per direction, no physical CAN.\n");
        command_sync_mode(session,"speed");
    }
    command_sync_enable(session);
    require(session.dual_enabled,"long CLI fake motors enabled");
    command_sync_zero(session);
    require(session.sync_zero_valid,"long CLI fixed zero recorded");
    int64_t actual=0;unsigned actions=0;double maximum_error=0;
    const auto began=yz_aim::Clock::now();
    do {
        for(bool extend : {true,false}) {
            if(!equal_speed_minute && !extend)break;
            const int duration=equal_speed_minute?3000:45000;
            command_sync_extend(session,{"300",std::to_string(duration)},extend);
            actual=std::chrono::duration_cast<std::chrono::milliseconds>(session.stop_a_at-session.sync_move_started).count();
            require(session.end_reason=="requested duration elapsed","long real CLI loop completes without false trip");
            require(actual>=duration && actual<duration+100,"long duration honored at stop-frame boundary");
            require(session.stop_b_at-session.stop_a_at<std::chrono::milliseconds(20),"long run stops both before ACK wait");
            require(!equal_speed_minute || (session.target_rpm_a==(extend?-300:300) && session.target_rpm_b==-session.target_rpm_a),"equal-speed long test retains exact mirror targets");
            maximum_error=std::max(maximum_error,session.max_error_deg);++actions;
        }
    }while(equal_speed_minute && yz_aim::Clock::now()-began<std::chrono::seconds(60));
    if(equal_speed_minute)require(bus.step_done && maximum_error>0.5,"minute test records position offset without changing speed or early stop");
    else require(session.max_error_deg<0.5,"long run accumulated position error stays below unchanged guard");
    const auto counts=trace.counts();
    require(counts.tx>1000 && counts.rx>1000 && counts.tx_error==0,"live tracing exercised throughout actual control loop");
    command_sync_disable(session);
    ::close(fd);
    std::printf("%s CLI with trace PASS: actions=%u wall_seconds=%.3f stop_ms=%lld max_error=%.6f TX=%llu RX=%llu\n",
        equal_speed_minute?"OFFLINE equal-speed minute":"45s",actions,
        std::chrono::duration<double>(yz_aim::Clock::now()-began).count(),
        (long long)actual,maximum_error,(unsigned long long)counts.tx,(unsigned long long)counts.rx);
}
} // namespace

int main(int argc,char** argv) {
    if(argc==2 && std::string(argv[1])=="--long") {test_45_second_cli_with_trace();return 0;}
    if(argc==2 && std::string(argv[1])=="--equal-speed-minute") {test_45_second_cli_with_trace(true);return 0;}
    test_command_node_change();
    test_sdo_request_spacing();
    test_shared_heartbeat_during_slow_other_axis();
    test_50_second_controller_models();
    test_coordinated_cancel_does_not_stop_one_axis_first();
    test_paired_start_does_not_wait_for_first_axis_ack();
    test_paired_start_does_not_wait_for_first_axis_ack(180);
    test_real_cli_loop(false,false);
    test_real_cli_loop(true,false);
    test_real_cli_loop(false,true);
    test_real_cli_loop(false,true,true);
    test_missing_start_ack(1U<<1);
    test_missing_start_ack(1U<<2);
    test_missing_start_ack((1U<<1)|(1U<<2));
    test_missing_start_ack(1U<<1,true);
    test_missing_start_ack(1U<<2,false,true);
    test_missing_start_ack(1U<<1,false,false,true);
    test_missing_start_ack(1U<<1,false,false,false,true);
    test_cli_repeat_after_missing_start_ack();
    std::printf("runtime_test: %d checks passed (offline, no physical CAN)\n",checks);
    return 0;
}
