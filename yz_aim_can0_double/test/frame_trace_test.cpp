#include "frame_trace.h"
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <cstdio>
#include <cstdlib>
int main() {
    const char* path="test/frame-trace-stress.log";
    int fd=::open(path,O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC,0600);
    if(fd<0)return 1;
    {
        yz_aim::FrameTrace trace("stress",true,fd);
        for(unsigned i=0;i<10000;++i)trace.frame("TX",yz_aim::Frame{0x601,8,{0x40,0x64,0x60}});
        if(trace.counts().tx!=10000 || trace.counts().trace_drop!=0)return 2;
    }
    ::close(fd);
    std::ifstream input(path);std::string line;unsigned count=0;
    while(std::getline(input,line))if(line.find("] TX ")!=std::string::npos)++count;
    if(count!=10000)return 3;
    std::printf("frame_trace_test: PASS (10000 records preserved, single producer/consumer)\n");
}
