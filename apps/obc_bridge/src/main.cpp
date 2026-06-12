#include "obcBridge.hpp"
#include "payloadConfig.hpp"

#ifdef DEBUG_WINDOW_ENABLED
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <limits.h>
#endif

int main()
{
#ifdef DEBUG_WINDOW_ENABLED
    if (getenv("PAYLOAD_DEBUG_WINDOW") == nullptr) {
        char exe_path[PATH_MAX];
        ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
        if (len != -1) {
            exe_path[len] = '\0';
            const char* disp_env = getenv("DISPLAY");
            std::string display = disp_env ? disp_env : ":0";
            std::string probe = "xdpyinfo -display " + display + " >/dev/null 2>&1";
            if (system(probe.c_str()) == 0) {
                std::string cmd = "DISPLAY=" + display +
                                  " PAYLOAD_DEBUG_WINDOW=1"
                                  " xterm -title \"OBC Bridge\""
                                  " -geometry 120x24+0+470"
                                  " -hold -e \""
                                  + std::string(exe_path) + "\"";
                int ret = system(cmd.c_str()); (void)ret;
                return 0;
            }
        }
    }
#endif

    ObcBridge obc_bridge;
    obc_bridge.run();
}