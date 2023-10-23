/*********************************************************************
 *
 * $Id: MqttBridge.c 57349 2023-10-20 11:31:48Z mvuilleu $
 *
 * MqttBridge: entry point
 *
 * - - - - - - - - - License information: - - - - - - - - -
 *
 *  Copyright (C) 2011 and beyond by Yoctopuce Sarl, Switzerland.
 *
 *  Yoctopuce Sarl (hereafter Licensor) grants to you a perpetual
 *  non-exclusive license to use, modify, copy and integrate this
 *  file into your software for the sole purpose of interfacing
 *  with Yoctopuce products.
 *
 *  You may reproduce and distribute copies of this file in
 *  source or object form, as long as the sole purpose of this
 *  code is to interface with Yoctopuce products. You must retain
 *  this notice in the distributed source file.
 *
 *  You should refer to Yoctopuce General Terms and Conditions
 *  for additional information regarding your rights and
 *  obligations.
 *
 *  THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 *  WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING
 *  WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO
 *  EVENT SHALL LICENSOR BE LIABLE FOR ANY INCIDENTAL, SPECIAL,
 *  INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 *  COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR
 *  SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT
 *  LIMITED TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR
 *  CONTRIBUTION, OR OTHER SIMILAR COSTS, WHETHER ASSERTED ON THE
 *  BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE), BREACH OF
 *  WARRANTY, OR OTHERWISE.
 *
 *********************************************************************/

#define __FILENAME__   "MqttBridge"
#include "MqttBridge.h"
#include "mqtt.h"
#include "yhash.h"
#include "yjson.h"

#include <stdio.h>
#include <stdarg.h>
#include <time.h>

#define SETTINGS_FILE_NAME      "mqttbridge.conf"

GlobalParameters Globalp;

static int ExitDaemon = 0;

u32 EnsureDaemonExitDelay = 20000; //20 seconds

yStrRef NetworkHubNameRef = INVALID_HASH_IDX;

static void* EnsureDaemonExitThread(void* dummy)
{
#ifdef WINDOWS_API
    Sleep(EnsureDaemonExitDelay);
#else
    usleep(EnsureDaemonExitDelay * 1000);
#endif
    exit(1);
}

void EnsureDaemonExit()
{
    ExitDaemon = 1;
    if (yCreateDetachedThreadNamed("EnsureDaemonExitThread", EnsureDaemonExitThread, NULL) < 0) {
        fprintf(stderr, "thread creation failed\n");
        exit(1);
    }
}

struct {
    const char* keyword;
    const char** settingsPtr;
    int flagVal;
} knownArguments[] = {
    { "registerHub", &Globalp.yoctohub_url, 0 },
    { "mqtt_broker", &Globalp.mqttbroker_host, 0 },
    { "mqtt_port", &Globalp.mqtt_port, 0 },
    { "mqtts_port", &Globalp.mqtts_port, 0 },
    { "mqtt_user", &Globalp.mqttUser, 0 },
    { "mqtt_pass", &Globalp.mqttPass, 0 },
    { "root_topic", &Globalp.rootTopic, 0 },
    { "allow_remote_control", NULL, MQTT_RW_MODE },
    { "no_instant_sensor_values", NULL, MQTT_NO_INSTANT_VALUE },
    { "no_retain_flag", NULL, MQTT_NO_RETAIN },
    { "cacert", &Globalp.mqttbroker_host, 0 },
    { "ignore_cert", NULL, IGNORE_CERT },
    { NULL, NULL, 0 }
};

static void Usage(int argc, char *argv[])
{
    printf("Usage: MqttBridge ...\n");
    printf("            --registerHub <yoctohub_ip_or_url>\n");
    printf("            --mqtt_broker <hostname_or_ip>\n");
    printf("            [--mqtt_port <port> | --mqtts_port <port>]\n");
    printf("            [--mqtt_user <username> --mqtt_pass <password>]\n");
    printf("            [--root_topic <mqtt_root_topic>]\n");
    printf("            [--allow_remote_control]\n");
    printf("            [--no_instant_sensor_values]\n");
    printf("            [--no_retain_flag]\n");
    printf("            [--cacert <ca_cert_file>]\n");
    printf("            [--ignore_cert]\n");
    printf("Special command-line options :\n");
    printf("   -c <conffile> : load config from JSON file to avoid long cmdline\n");
    printf("      (config file uses key names identical to the options listed above)\n");
    printf("   -g <logfile>  : log debug infos into a <logfile>\n");
    printf("   -i : install as a service (with the arguments specified)\n");
    printf("   -u : uninstall as a service\n");
    printf("   -d : run as a daemon (when running as a service on Unix) \n");
    printf("   -v : show the version of the executable\n");
    exit(0);
}

static void PrintFatalError(int error, const char *yapierr)
{
    fprintf(stderr, "********************************************************************************\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Yoctopuce API initialization failed :\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "%s\n", yapierr);
    fprintf(stderr, "\n");
    fprintf(stderr, "********************************************************************************\n");
}

int loadConfigFile(const char* filename)
{
    char                value[512], *configFile;
    yJsonStateMachine   j;
    yJsonRetCode        jstate = YJSON_NEED_INPUT;
    u16                 res = 0, depth = 0, opt;

    configFile = loadtext(filename);
    if (!configFile) return -1;
    ylogf("Loading configuration from %s\n", filename);

    memset(&j, 0, sizeof(j));
    j.st = YJSON_START;
    j.src = configFile;
    j.end = configFile + strlen(configFile);
    jstate = yJsonParse(&j);
    while (jstate == YJSON_PARSE_AVAIL) {
        switch (j.st) {
        case YJSON_PARSE_STRUCT:
            depth += (j.token[0] == '{' ? 1 : -1);
            if (!depth) {
                // exiting nested API node, stop parsing file
                jstate = YJSON_SUCCESS;
                continue;
            }
            break;
        case YJSON_PARSE_MEMBNAME:
            for (opt = 0; knownArguments[opt].keyword; opt++) {
                if (ystrcmp(j.token, knownArguments[opt].keyword) == 0) {
                    if (knownArguments[opt].settingsPtr) {
                        jstate = yJsonParse(&j);
                        ystrcpy(value, sizeof(value), j.token);
                        while (jstate == YJSON_PARSE_AVAIL && (j.next == YJSON_PARSE_STRINGCONT || j.next == YJSON_PARSE_STRINGCONTQ)) {
                            jstate = yJsonParse(&j);
                            if (jstate != YJSON_PARSE_AVAIL) break;
                            ystrcat(value, sizeof(value), j.token);
                        }
                        // ylogf("- set [%s] to '%s'\n", knownArguments[opt].keyword, value);
                        *(knownArguments[opt].settingsPtr) = ystrdup(value);
                    } else {
                        jstate = yJsonParse(&j);
                        if (j.token[0] != '0' && j.token[0] != 'F' && j.token[0] != 'f') {
                            // not set to 0 or False
                            // ylogf("- set flag [%s]\n", knownArguments[opt].keyword);
                            Globalp.flags |= knownArguments[opt].flagVal;
                        }
                    }
                    break;
                }
            }
            if (!knownArguments[opt].keyword) {
                ylogf("- unrecognized setting [%s]", j.token);
                res = -1;
                yJsonSkip(&j, 1);
            }
            break;
        default:;
        }
        jstate = yJsonParse(&j);
    }
    free(configFile);
    return res;
}


int ParseArguments(int argc, char *argv[])
{
    int i, opt, argReguired = 0;
#if defined(WINDOWS_API) || defined(LINUX_API)
    int res, install = 0;
    char errmsg[YOCTO_ERRMSG_LEN];
#endif
    memset(&Globalp, 0, sizeof(GlobalParameters));

    for (i = 1; i < argc; i++) {
        if (argv[i][0] != '-' && argv[i][0] != '/') {
            fprintf(stderr, "MqttBridge: unknown option %s\n", argv[i]);
            fprintf(stderr, "Try \"MqttBridge -h\" for more information.\n");
            exit(1);
        }
        switch (argv[i][1]) {
        case 'd':
            Globalp.flags |= VHUB_RUN_IN_DEAMON;
            break;
        case 'g':
            if (i + 1 < argc) {
                Globalp.logfile = argv[++i];
            } else {
                argReguired = 1;
            }
            break;
        case 'c':
            if (i + 1 < argc) {
                Globalp.settingsfile = argv[++i];
            } else {
                argReguired = 1;
            }
            break;
#if defined(WINDOWS_API) || defined(LINUX_API)
        case 'i':
            install = 1;
            break;
        case 'u':
            res = InstallService(0, argv[0], "", errmsg);
            if (res < 0) {
                fprintf(stderr, "ERROR: unable to uninstall service (%s)\n", errmsg);
            }
            exit(0);
#endif
        case 'v': {
            const char *apiversion, *apidate;
            yapiGetAPIVersion(&apiversion, &apidate);
            printf("Version %s (%s)\n", apiversion, apidate);
            exit(0);
        }
        case 'h':
            Usage(argc, argv);
        case '-':
            // double -- options
            if (ystrcmp(argv[i], "--help") == 0) {
                Usage(argc, argv);
            }
            if (ystrcmp(argv[i], "--version") == 0) {
                const char* apiversion, * apidate;
                yapiGetAPIVersion(&apiversion, &apidate);
                printf("MqttBridge version %s (%s)\n", apiversion, apidate);
                exit(0);
            }
            for (opt = 0; knownArguments[opt].keyword; opt++) {
                if (ystrcmp(argv[i] + 2, knownArguments[opt].keyword) == 0) {
                    if (knownArguments[opt].settingsPtr) {
                        if (i + 1 >= argc) {
                            fprintf(stderr, "MqttBridge: --%s option requires an argument\n", knownArguments[opt].keyword);
                            fprintf(stderr, "Try \"MqttBridge -h\" for more information.\n");
                            exit(1);
                        }
                        *(knownArguments[opt].settingsPtr) = argv[++i];
                    } else {
                        Globalp.flags |= knownArguments[opt].flagVal;
                    }
                    break;
                }
            }
            if (knownArguments[opt].keyword) {
                break; // argument handled
            }
        // no break on purpose
        default:
            fprintf(stderr, "MqttBridge: unknown option %s\n", argv[i]);
            fprintf(stderr, "Try \"MqttBridge -h\" for more information.\n");
            exit(1);
        }
        if (argReguired) {
            fprintf(stderr, "MqttBridge: option -%c requires an argument\n", argv[i][1]);
            fprintf(stderr, "Try \"MqttBridge -h\" for more information.\n");
            exit(1);
        }
    }

#if defined(WINDOWS_API) || defined(LINUX_API)
    if (install) {
        char allargs[2048] = "";

        if (Globalp.settingsfile) {
            ystrcat(allargs, sizeof(allargs), " -c ");
            ystrcat(allargs, sizeof(allargs), relativeToAbsolutePath(Globalp.settingsfile));
        }
        if (Globalp.logfile) {
            ystrcat(allargs, sizeof(allargs), " -g ");
            ystrcat(allargs, sizeof(allargs), relativeToAbsolutePath(Globalp.logfile));
        }
        for (opt = 0; knownArguments[opt].keyword; opt++) {
            if (knownArguments[opt].settingsPtr) {
                if (*(knownArguments[opt].settingsPtr)) {
                    ystrcat(allargs, sizeof(allargs), " --");
                    ystrcat(allargs, sizeof(allargs), knownArguments[opt].keyword);
                    ystrcat(allargs, sizeof(allargs), " ");
                    ystrcat(allargs, sizeof(allargs), *(knownArguments[opt].settingsPtr));
                }
            } else {
                if (Globalp.flags & knownArguments[opt].flagVal) {
                    ystrcat(allargs, sizeof(allargs), " --");
                    ystrcat(allargs, sizeof(allargs), knownArguments[opt].keyword);
                }
            }
        }
        printf("install with args [%s]\n", allargs);
        res = -1;// InstallService(1, argv[0], allargs, errmsg);
        if (res < 0) {
            fprintf(stderr, "ERROR: unable to install service : %s\n", errmsg);
        }
        exit(0);
    }
#endif

    if (Globalp.settingsfile) {
        if (loadConfigFile(Globalp.settingsfile) < 0) {
            fprintf(stderr, "Failed to read config file %s\n", relativeToAbsolutePath(Globalp.settingsfile));
            exit(1);
        }
    }

    if (!Globalp.yoctohub_url) {
        fprintf(stderr, "No Yoctopuce hub specified, cannot start!\n");
        fprintf(stderr, "Try \"MqttBridge -h\" for more information.\n");
        exit(1);
    }
    if (!Globalp.mqttbroker_host) {
        fprintf(stderr, "No MQTT broker specified, cannot start!\n");
        fprintf(stderr, "Try \"MqttBridge -h\" for more information.\n");
        exit(1);
    }
    if (Globalp.mqtts_port) {
        Globalp.mqttbroker_port = (u16)atoi(Globalp.mqtts_port);
        Globalp.flags |= MQTT_USE_SSL;
    }
    else if (Globalp.mqtt_port) {
        Globalp.mqttbroker_port = (u16)atoi(Globalp.mqtt_port);
    }
    else {
        Globalp.mqttbroker_port = 1883;
    }
    if (!Globalp.mqttbroker_port) {
        fprintf(stderr, "Invalid MQTT port, cannot start!\n");
        fprintf(stderr, "Try \"MqttBridge -h\" for more information.\n");
        exit(1);
    }

    return 1;
}

void ylogf(const char* fmt, ...)
{
    va_list     args;
    char        buffer[2048], buffer2[64];
    char* tbuffer = buffer2 + 1, * thrptr;
    struct tm   timeinfo;
    time_t      rawtime;
    size_t      len;
    int         threadIdx;

    time(&rawtime);
    threadIdx = yThreadIndex();
    ylocaltime(&timeinfo, &rawtime);
    strftime(tbuffer, sizeof(buffer2), "[%Y-%m-%d %H:%M:%S] ", &timeinfo);
    thrptr = tbuffer + strlen(tbuffer);

    va_start(args, fmt);
#ifdef WINDOWS_API
    sprintf_s(thrptr, sizeof(buffer2) - (thrptr - buffer2), "(%02x) ", threadIdx);
    vsprintf_s(buffer, sizeof(buffer), fmt, args);
#else
    sprintf(thrptr, "(%02x) ", threadIdx);
    vsprintf(buffer, fmt, args);
#endif
    va_end(args);
    len = strlen(buffer);
    if (len == 0) return;

    if (Globalp.logfile) {
        FILE* f;
        if (YFOPEN(&f, Globalp.logfile, "a") == 0) {
            fwrite(tbuffer, 1, strlen(tbuffer), f);
            fwrite(buffer, 1, len, f);
            fclose(f);
        } else {
#ifdef WINDOWS_API
            char errorbuf[256];
            strerror_s(errorbuf, sizeof(errorbuf), errno);
            fprintf(stderr, "%sUnable to save log to file %s (%s)\n", tbuffer, Globalp.logfile, errorbuf);
#else
            fprintf(stderr, "%sUnable to save log to file %s (%s)\n", tbuffer, Globalp.logfile, strerror(errno));
#endif
        }
    }
    printf("%s%s", tbuffer, buffer);
}

static int nbCtrlC = 1;

void CtrlCHandler(int dummy)
{
    if (nbCtrlC--) {
        EnsureDaemonExit();
    } else {
        exit(1);
    }
}

static void LogFn(const char* msg, u32 len)
{
    if (strstr(msg, "ASSERT")) {
        fprintf(stderr, "%s", msg);
    }
    ylogf("%s", msg);
}

static void DeviceArrivalFn(YAPI_DEVICE devdescr)
{
    yDeviceSt infos;

    yapiGetDeviceInfo(devdescr, &infos, NULL);
    ylogf("Device connected: %s\n", infos.serial);

    // The very first arrival callback comes from the hub itself
    if (NetworkHubNameRef == INVALID_HASH_IDX) {
        SerialRef = yHashPutStr(infos.serial);
        NetworkHubNameRef = yHashPutStr(infos.logicalname);
    }
    mqtt_trigger_announce();
}

void DeviceRemovalFn(YAPI_DEVICE devdescr)
{
    yDeviceSt infos;
    yStrRef serialref;

    yapiGetDeviceInfo(devdescr, &infos, NULL);
    serialref = yHashTestStr(infos.serial);

    ylogf("Device disconnected: %s\n", infos.serial);
    mqtt_push_unplug(serialref);
}

void DeviceUpdateFn(YAPI_DEVICE devdescr)
{
    yDeviceSt infos;
    yStrRef serialref;
    int devydx;

    yapiGetDeviceInfo(devdescr, &infos, NULL);
    serialref = yHashTestStr(infos.serial);
    devydx = wpGetDevYdx(serialref);

    //ylogf("Device update for %s (devYdx=%d)\n", infos.serial, devydx);
    mqtt_setFlag(NB_MAX_DEVICES + devydx, 0xf);
    mqtt_trigger_announce();
}

void DeviceConfigChangeFn(YAPI_DEVICE devdescr)
{
    yDeviceSt infos;
    yStrRef serialref;
    int devydx;

    yapiGetDeviceInfo(devdescr, &infos, NULL);
    serialref = yHashTestStr(infos.serial);
    devydx = wpGetDevYdx(serialref);

    //ylogf("Device config change for %s (devYdx=%d)\n", infos.serial, devydx);
    mqtt_setFlag(NB_MAX_DEVICES + devydx, -1);
}

void BeaconCallbackFn(YAPI_DEVICE devdescr, int beacon)
{
    yDeviceSt infos;
    yStrRef serialref;
    int devydx;

    yapiGetDeviceInfo(devdescr, &infos, NULL);
    serialref = yHashTestStr(infos.serial);
    devydx = wpGetDevYdx(serialref);

    //ylogf("Device beacon changed for %s (devYdx=%d): beacon=%d\n", infos.serial, devydx, beacon);
    mqtt_setFlag(NB_MAX_DEVICES + devydx, 0xf);
}

void FunctionUpdateFn(YAPI_FUNCTION fundescr, const char* value)
{
    char serialNumber[YOCTO_SERIAL_LEN];
    char functionId[YOCTO_FUNCTION_LEN];
    char errmsg[128];
    yStrRef serialref;
    yStrRef funcId;
    int devydx;
    int funydx;

    // callback may be invoked with value == NULL to notify of new logical name,
    // but we will catch this change using the config change callback
    if (!value) return; 

    yapiGetFunctionInfo(fundescr, NULL, serialNumber, functionId, NULL, NULL, errmsg);
    serialref = yHashTestStr(serialNumber);
    funcId = yHashTestStr(functionId);
    devydx = wpGetDevYdx(serialref);
    funydx = ypSearchByDevYdx((u8)devydx, funcId);

    //ylogf("Function update for %s.%s (devYdx=%d, funYdx=%d): %s\n", serialNumber, functionId, devydx, funydx, value);
    mqtt_setFlag((u16)devydx, (s16)funydx);
}

void TimedReportFn(YAPI_FUNCTION fundescr, double timestamp, const u8* data, u32 len, double duration)
{
    char serialNumber[YOCTO_SERIAL_LEN];
    char functionId[YOCTO_FUNCTION_LEN];
    char errmsg[128];
    yStrRef serialref;
    yStrRef funcId;
    int devydx;
    int funydx;

    // discard anything but timed reports V2
    if (len-- < 2 || *data++ != 2) return;

    yapiGetFunctionInfo(fundescr, NULL, serialNumber, functionId, NULL, NULL, errmsg);
    serialref = yHashTestStr(serialNumber);
    funcId = yHashTestStr(functionId);
    devydx = wpGetDevYdx(serialref);
    funydx = ypSearchByDevYdx((u8)devydx, funcId);

    //ylogf("Timed reports for %s.%s (devYdx=%d, funYdx=%d)\n", serialNumber, functionId, devydx, funydx);
    if (len <= 5) {
        // immediate report
        mqtt_push_not_item((u8)devydx, (u8)funydx, 0, 0, data, len);
    } else {
        // averaged report: avg,avg-min,max-avg
        s32 avgval;
        u8 detail = *data++;
        len = 1 + (detail & 3);
        avgval = mqtt_push_not_item((u8)devydx, (u8)funydx, 0, 0, data, len);
        data += len;
        len = 1 + ((detail >> 2) & 3);
        mqtt_push_not_item((u8)devydx, (u8)funydx, 0x10, avgval, data, len);
        data += len;
        len = 1 + ((detail >> 4) & 3);
        mqtt_push_not_item((u8)devydx, (u8)funydx, 0x20, avgval, data, len);
    }
}

int TunnelMain(char *errmsg)
{
    const char *apiversion, *apidate;
    char yapierr[YOCTO_ERRMSG_LEN];
    int res;

    yapiGetAPIVersion(&apiversion, &apidate);
#ifdef YBUILD_PATCH_WITH_BUILD
    ylogf("Yoctopuce MqttBridge %s (%s %s)\n", apiversion, __DATE__, __TIME__);
#else
    ylogf("Yoctopuce MqttBridge %s (dated %s)\n", apiversion, __DATE__);
#endif
    mqtt_init();

    // Init YAPI (USB)
    res = yapiInitAPIEx(Y_DETECT_NONE, NULL, NULL, yapierr);
    if (YISERR(res)) {
        PrintFatalError(res, yapierr);
        return -1;
    }
    yapiRegisterLogFunction(LogFn);
    yapiRegisterDeviceArrivalCallback(DeviceArrivalFn);
    yapiRegisterDeviceRemovalCallback(DeviceRemovalFn);
    yapiRegisterDeviceChangeCallback(DeviceUpdateFn);
    yapiRegisterDeviceConfigChangeCallback(DeviceConfigChangeFn);
    yapiRegisterBeaconCallback(BeaconCallbackFn);
    yapiRegisterFunctionUpdateCallback(FunctionUpdateFn);
    yapiRegisterTimedReportCallback(TimedReportFn);

    if (Globalp.flags & IGNORE_CERT) {
        res = yapiSetNetworkSecurityOptions(YSSL_NO_HOSTNAME_CHECK | YSSL_NO_TRUSTED_CA_CHECK | YSSL_NO_EXPIRATION_CHECK, yapierr);
        if (res < 0) {
            PrintFatalError(YAPI_IO_ERROR, "Failed to setup security options");
            return -1;
        }
    }
    if (Globalp.ca_certiffile) {
        char *cert_data = NULL;
        ylogf("Load client certificate chain from %s file.\n", Globalp.ca_certiffile);
        cert_data = loadtext(Globalp.ca_certiffile);
        if(!cert_data) {
            PrintFatalError(YAPI_IO_ERROR, "Unable to load file");
            return -1;
        }
        int res = yTcpAddClientCertificateSSL((u8 *)cert_data, (u32)strlen(cert_data), yapierr);
        if (res < 0) {
            PrintFatalError(YAPI_IO_ERROR, "Invalid client certificate chain");
            return -1;
        }
    }

    res = yapiPreregisterHub(Globalp.yoctohub_url, yapierr);
    if (YISERR(res)) {
        PrintFatalError(res, yapierr);
        return -1;
    }
    ylogf("Trying to connect to Yoctopuce hub\n");

    while (!ExitDaemon) {
        yapiUpdateDeviceList(0, yapierr);
        while (mqtt_run()) {
            yapiHandleEvents(yapierr);
        }
        yapiSleep(10, yapierr);
    }
    return 0;
}
