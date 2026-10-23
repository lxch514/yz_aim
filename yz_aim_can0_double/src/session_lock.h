#pragma once
#include <filesystem>
#include <string>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace yz_aim {
// Prevent two copies of this project from issuing interleaved SDO transactions.
class SessionLock {
public:
    ~SessionLock() { if (fd_ >= 0) ::close(fd_); }
    bool acquire(const std::filesystem::path& directory, const std::string& interface, std::string& error) {
        if (interface.empty() || interface.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") != std::string::npos) {
            error="接口名无效";return false;
        }
        std::error_code ec;
        std::filesystem::create_directories(directory,ec);
        if (ec) { error="无法创建本目录会话锁："+ec.message();return false; }
        fd_=::open((directory/(interface+".lock")).c_str(),O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
        if (fd_<0 || ::flock(fd_,LOCK_EX|LOCK_NB)!=0) {
            error="本目录已有程序占用 "+interface+"，请回到原控制终端操作，或先正常退出原程序";
            return false;
        }
        return true;
    }
private:
    int fd_=-1;
};
}
