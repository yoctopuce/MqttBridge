/*********************************************************************
 *
 * $Id: linStartup.c 57313 2023-10-20 09:00:00Z seb $
 *
 * Entry point for the application
 *
 * - - - - - - - - - License information: - - - - - - - - -
 *
 * Copyright (C) 2011 and beyond by Yoctopuce Sarl, Switzerland.
 *
 * 1) If you have obtained this file from www.yoctopuce.com using
 *    a valid customer account established in your proper name,
 *    Yoctopuce Sarl (hereafter Licensor) licenses to you (hereafter
 *    Licensee) the right to use, modify, copy, and integrate this
 *    source file into your own solution for the sole purpose of
 *    interfacing a Yoctopuce product integrated into Licensee's
 *    solution.
 *
 *    You should refer to the license agreement accompanying this
 *    Software for additional information regarding your rights
 *    and obligations.
 *
 *    THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 *    WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING
 *    WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS
 *    FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO
 *    EVENT SHALL LICENSOR BE LIABLE FOR ANY INCIDENTAL, SPECIAL,
 *    INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 *    COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR
 *    SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT
 *    LIMITED TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR
 *    CONTRIBUTION, OR OTHER SIMILAR COSTS, WHETHER ASSERTED ON THE
 *    BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE), BREACH OF
 *    WARRANTY, OR OTHERWISE.
 *
 * 2) If you have obtained this file from any other source, you
 *    are not entitled to use it, read it or create any derived
 *    material. You should delete this file immediately.
 *
 *********************************************************************/

#define __FILENAME__   "linStartup"
#include "MqttBridge.h"

#ifdef LINUX_API

#include <err.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>

#define PIDFILE "/var/run/mqttbridge.pid"

const char * systemd_scipt=
"[Unit]\n"
"Description=Yoctopuce MqttBridge\n"
"After=network.target\n"
"\n"
"[Service]\n"
"ExecStart=%s %s\n"
"Type=simple\n"
"\n"
"[Install]\n"
"WantedBy=multi-user.target\n";

static int InstallSystemD(int install, const char *exe, const char *exe_args, char *errmsg)
{
    if (access("/etc/systemd/system/", W_OK) < 0){
        return YERRMSG(YAPI_UNAUTHORIZED,"Permission denied");
    }

    if (install) {
        char buffer[1024];
        char self[PATH_MAX];;

        int size = readlink("/proc/self/exe", self, PATH_MAX);

        if (size < 0 ) {
            // unable to get absolute path with /proc/self/exe use args[0]
            // and hope for the best
        } else{
            // use absolut executable path
            exe = self;
        }
        YSPRINTF(buffer, 1024, systemd_scipt, exe, exe_args);

        if (write_text_file("/etc/systemd/system/mqttbridge.service", buffer, ystrlen(buffer), errmsg) < 0) {
            return YERR(YAPI_IO_ERROR);
        }

        if (system("systemctl daemon-reload") < 0) {
            return YERRMSG(YAPI_IO_ERROR,"command \"systemctl daemon-reload\" failed");
        }
        if (system("systemctl start mqttbridge.service") < 0) {
            return YERRMSG(YAPI_IO_ERROR,"command \"systemctl start mqttbridge.service\" failed");
        }
        if (system("systemctl enable mqttbridge.service") < 0) {
            return YERRMSG(YAPI_IO_ERROR,"command \"systemctl enable mqttbridge.service\" failed");
        }
    } else {
        if (system("systemctl stop mqttbridge.service") < 0) {
            return YERRMSG(YAPI_IO_ERROR,"command \"systemctl start mqttbridge.service\" failed");
        }
        if (system("systemctl disable mqttbridge.service") < 0) {
            return YERRMSG(YAPI_IO_ERROR,"command \"systemctl enable mqttbridge.service\" failed");
        }
    }
    return YAPI_SUCCESS;
}

int InstallService(int install, const char *exe, const char *args, char *errmsg)
{
    if (access("/etc/systemd/system.conf", F_OK) == 0){
        return InstallSystemD(install, exe, args,errmsg);
    }
    return YERRMSG(YAPI_NOT_SUPPORTED,"Unsupported init system");
}

int main(int argc, char* argv[])
{
    char    errmsg[YOCTO_ERRMSG_LEN*2];

    signal(SIGPIPE,SIG_IGN);
    signal(SIGINT, CtrlCHandler);
    signal(SIGKILL, CtrlCHandler);

    ParseArguments(argc,argv);
    if(Globalp.flags & VHUB_RUN_IN_DEAMON) {
#ifdef CREATE_PIDFILE
        struct  pidfh *pfh;
        pid_t   otherpid, childpid;
        pfh = pidfile_open(PIDFILE, 0600, &otherpid);
        if (pfh == NULL) {
            if (errno == EEXIST) {
                errx(EXIT_FAILURE, "MqttBridge already running, pid: %jd.",(intmax_t)otherpid);
            }
            /* If we cannot create pidfile from other reasons, only warn. */
            warn("Cannot open or create pidfile");
        }
        if (daemon(0, 0) == -1) {
            warn("Cannot daemonize");
            pidfile_remove(pfh);
            exit(EXIT_FAILURE);
        }
        pidfile_write(pfh);
        TunnelMain(errmsg);
        pidfile_remove(pfh);
        exit(EXIT_SUCCESS);
#else
        if (daemon(0, 0) < 0) {
            exit(1);
        }
#endif
    }

    return TunnelMain(errmsg);
}

#endif
