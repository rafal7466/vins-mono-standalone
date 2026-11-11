#include "ros_compat.h"
#include <mutex>

namespace vins_log
{
int level = 1;
FILE *file = nullptr;
static std::mutex mtx;
static const char *tag(int l) { return l <= 0 ? "[DEBUG] " : l == 1 ? "[INFO] " : l == 2 ? "[WARN] " : "[ERROR] "; }

void vprint(int lvl, const char *fmt, ...)
{
    if (lvl < level)
        return;
    char buf[4096];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    sprint(lvl, buf);
}

void sprint(int lvl, const std::string &msg)
{
    if (lvl < level)
        return;
    std::lock_guard<std::mutex> lk(mtx);
    FILE *o = lvl >= 2 ? stderr : stdout;
    std::fprintf(o, "%s%s\n", tag(lvl), msg.c_str());
    if (file)
    {
        std::fprintf(file, "%s%s\n", tag(lvl), msg.c_str());
        std::fflush(file);
    }
}
} // namespace vins_log
