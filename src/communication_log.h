#pragma once
#include "frame_trace.h"
#include "session_lock.h"
#include <filesystem>
#include <memory>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
namespace yz_aim {
class CommunicationLog {
public:
    bool open(const std::string& label,const std::string& marker,std::string& error) {
        std::error_code ec;
        auto root=std::filesystem::canonical("/proc/self/exe",ec).parent_path();
        for(unsigned i=0;!ec && i<6;++i,root=root.parent_path())
            if(std::filesystem::is_regular_file(root/marker) && std::filesystem::is_regular_file(root/"build.sh"))break;
        if(ec || !std::filesystem::is_regular_file(root/marker)) {error="无法定位本程序目录";return false;}
        if(!ownership_.acquire(root/".runtime",label,error))return false;
        std::filesystem::create_directories(root/"logs",ec);
        if(ec){error="无法创建通讯日志目录："+ec.message();return false;}
        path_=root/"logs"/("can-frames-"+std::to_string(std::time(nullptr))+"-"+std::to_string(::getpid())+".log");
        int fd=::open(path_.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_APPEND|O_CLOEXEC,0600);
        if(fd<0){error="无法创建通讯日志";return false;}
        trace_=std::make_unique<FrameTrace>(label,true,fd);::close(fd);return true;
    }
    FrameTrace* trace() {return trace_.get();}
    const std::filesystem::path& path() const {return path_;}
private:
    SessionLock ownership_;
    std::filesystem::path path_;
    std::unique_ptr<FrameTrace> trace_;
};
}
