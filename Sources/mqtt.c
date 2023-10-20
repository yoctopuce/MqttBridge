/*********************************************************************
 *
 * $Id: mqtt.c 57313 2023-10-20 09:00:00Z seb $
 *
 * basic mqtt communication support
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

#define __FILENAME__   "mqtt"
#include "MqttBridge.h"
#include "yproto.h"
#include "yhash.h"
#include "mqtt.h"
#include "request.h"

#define CB_BUFF_SIZE                            8190

#define LocalhostOpen(conn, mode, errmsg)       yoctohubOpenReq(errmsg)
#define LocalhostGetInputFifo(conn)             yoctohubGetInputFifo()
#define LocalhostGetOutputFifo(conn)            yoctohubGetOutputFifo()
#define LocalhostIsDone(conn)                   yoctohubReqIsDone()
#define LocalhostDisconnect(conn)               yoctohubCloseReq()

#define CALLBACK_TIMEOUT                        YIO_DEFAULT_TCP_TIMEOUT
#define ytime()                                 ((u32)yapiGetTickCount())
#define YFSTR(id, str)                          str

typedef enum {
    CB_IDLE = 0,
    CB_MQTT_TRY,
    CB_MQTT_WAIT_CONNECT,
    CB_MQTT_ANNOUNCE,
    CB_MQTT_CONNECTED,
    CB_FAILED
} CBState;

typedef enum {
    MQTT_PKT_RESERVED = 0,
    MQTT_PKT_CONNECT = 1,
    MQTT_PKT_CONNACK = 2,
    MQTT_PKT_PUBLISH = 3,
    MQTT_PKT_PUBACK = 4,
    MQTT_PKT_PUBREC = 5,
    MQTT_PKT_PUBREL = 6,
    MQTT_PKT_PUBCOMP = 7,
    MQTT_PKT_SUBSCRIBE = 8,
    MQTT_PKT_SUBACK = 9,
    MQTT_PKT_UNSUBSCRIBE = 10,
    MQTT_PKT_UNSUBACK = 11,
    MQTT_PKT_PINGREQ = 12,
    MQTT_PKT_PINGRESP = 13,
    MQTT_PKT_DISCONNECT = 14
} MQTT_PKT_TYPE;

struct {
    CBState     state;
    int         lo_open : 1;
    YSOCKET_MULTI strym;                // the socket to connect to mqtt broker
    yTime       timeref;
    char        *errmsg;
    yCRITICAL_SECTION mqtt_fifo_cs;
    char        buf[512];
    u32         contentLength;
    u16         writepos;
    struct {
        yTime nextAnnounce;
        const char* rootTopic;          // configuration option
        u16 rootTopicLen;
        union {
            struct {
                u16 disableIv : 1;      // configuration option
                u16 noRetain : 1;       // configuration option
                u16 rwMode : 1;         // configuration option
                u16 mqttIdle : 1;
            };
            u16 flags;
        };
        s16 devYdx;
        s16 funYdx;
        u16 recIdx;
        u16 subState;
        u16 tailBufLen;
        s16 httpBufLen;
        u16 httpBufPos;
        u16 head_ofs;
        u16 tail_ofs;
        u16 used;
    } mqtt_ctx;
} CbCtx;

struct {
    u16         len;
    union {
        char    buff[CB_BUFF_SIZE];
        u16     wbuff[CB_BUFF_SIZE / 2];
    };
} cbBuff;

#define __eds__ 

static void EdsMoveTo(u16 ofs, const u8* in, int len)
{
    if (ofs >= CB_BUFF_SIZE) {
        return;
    }
    if (ofs + len > CB_BUFF_SIZE) {
        len = CB_BUFF_SIZE - ofs;
    }
    while (len--) {
        cbBuff.buff[ofs++] = *in++;
    }
}

static s16 EdsLoadFromFifo(yFarFifoBuf* fifo, u8* buf, u16 bufLen, u16 edsBase, u16 edsMaxLen, u16* lenPtr, char *errmsg)
{
    u16 len = yFarFifoGetUsed(fifo);
    if (!*lenPtr) {
        if (len < 7) {
            if (ytime() - CbCtx.timeref > CALLBACK_TIMEOUT) {
                CbCtx.errmsg = errmsg;
                CbCtx.buf[0] = 0;
                CbCtx.state = CB_FAILED;
            }
            return -1;
        }
        // drop "OK\r\n\r\n" header
        yPopFarFifo(fifo, buf, 6);
        if (memcmp(buf, "OK\r\n\r\n", 6) != 0) {
            // download as much as possible of the HTTP reply to log error
            len = ySeekFarFifo(fifo, (u8*)"\r\n", 2, 0, 30, 0);
            if (len == 0xffff) {
                len = yFarFifoGetUsed(fifo);
            }
            if (len > bufLen - 7) {
                len = bufLen - 7;
            }
            yPopFarFifo(fifo, buf + 6, len);
            len += 6;
            CbCtx.errmsg = errmsg;
            CbCtx.state = CB_FAILED;
            return -1;
        }
    }
    while ((len = yFarFifoGetUsed(fifo)) > 0) {
        if (len > bufLen) {
            len = bufLen;
        }
        yPopFarFifo(fifo, buf, len);
        if (*lenPtr + len > edsMaxLen) {
            len = edsMaxLen - *lenPtr;
        }
        EdsMoveTo(edsBase + *lenPtr, buf, len);
        *lenPtr += len;
    }
    return 0;
}

static void EdsMoveFrom(u16 ofs, u8* out, int len)
{
    while (len--) {
        *out++ = cbBuff.buff[ofs++];
    }
}

// Helper to save memory
static int ypGetDevAttributesByYdx(u8 devydx, yStrRef* serial, yStrRef* devname)
{
    return ypGetAttributesByYdx(devydx, 0xf, serial, devname, NULL, NULL, NULL, NULL, NULL);
}

// Buffer for building MQTT announcement message (aka Home Automation integration)
#define MQTT_HTTPBUF_START  (NB_MAX_DEVICES * 4)
#define MQTT_HTTPBUF_SIZE   1024
// MQTT-specific circular buffer for timed reports
#define MQTT_TIMEREP_START  (NB_MAX_DEVICES * 4 + MQTT_HTTPBUF_SIZE)
#define MQTT_TIMEREP_SIZE   (CB_BUFF_SIZE - MQTT_TIMEREP_START)
#define MQTT_TO_EDS(ofs, in, len) EdsMoveTo((ofs) + MQTT_TIMEREP_START, in, len )
#define MQTT_FROM_EDS(ofs, out, len) EdsMoveFrom((ofs) + MQTT_TIMEREP_START, out, len )

typedef struct {
    yStrRef serialref;
    yStrRef nameref;
} unplugData;

typedef struct {
    u8          pseudoDevYdx;   // 0
    u8          pseudoFunYdx;   // 0xff
    unplugData  child;
} unplugRecord;

static int mqtt_drop()
{
    int cofs;
    //ASSERT(CbCtx.mqtt_ctx.used >= 6);
    cofs = CbCtx.mqtt_ctx.head_ofs + 6;
    if (cofs > MQTT_TIMEREP_SIZE) {
        cofs = cofs - MQTT_TIMEREP_SIZE;
    }
    CbCtx.mqtt_ctx.head_ofs = cofs;
    CbCtx.mqtt_ctx.used -= 6;
    return CbCtx.mqtt_ctx.used;
}

static void mqtt_push_data(const u8* data, int datalen)
{
    u16 tail;
    u16 freespace;
    yEnterCriticalSection(&CbCtx.mqtt_fifo_cs);

    tail = CbCtx.mqtt_ctx.tail_ofs;
    freespace = MQTT_TIMEREP_SIZE - CbCtx.mqtt_ctx.used;
    if (datalen > freespace) {
        // drop one notification
        mqtt_drop();
    }
    //append data to our fifo buffer
    if (tail + datalen <= MQTT_TIMEREP_SIZE) {
        MQTT_TO_EDS(tail, data, datalen);
        tail += datalen;
        if (tail == MQTT_TIMEREP_SIZE)
            tail = 0;
    }
    else {
        u16 cplen = (u16)(MQTT_TIMEREP_SIZE - tail);
        MQTT_TO_EDS(tail, data, cplen);
        tail = datalen - cplen;
        MQTT_TO_EDS(0, data + cplen, tail);
    }
    CbCtx.mqtt_ctx.tail_ofs = tail;
    CbCtx.mqtt_ctx.used += datalen;
    yLeaveCriticalSection(&CbCtx.mqtt_fifo_cs);
}

static int mqtt_fifo_size()
{
    int res;
    yEnterCriticalSection(&CbCtx.mqtt_fifo_cs);
    res = CbCtx.mqtt_ctx.used;
    yLeaveCriticalSection(&CbCtx.mqtt_fifo_cs);
    return res;
}

static u8 mqtt_peek_byte(int ofs)
{
    int cofs;
    u8 res;
    yEnterCriticalSection(&CbCtx.mqtt_fifo_cs);
    cofs = ofs + CbCtx.mqtt_ctx.head_ofs;
    if (cofs > MQTT_TIMEREP_SIZE) {
        cofs = cofs - MQTT_TIMEREP_SIZE;
    }
    res = cbBuff.buff[cofs + MQTT_TIMEREP_START];
    yLeaveCriticalSection(&CbCtx.mqtt_fifo_cs);
    return res;
}

static void mqtt_peek(u8* data, int ofs, int len)
{
    int cofs;
    yEnterCriticalSection(&CbCtx.mqtt_fifo_cs);
    cofs = ofs + CbCtx.mqtt_ctx.head_ofs;
    if (cofs > MQTT_TIMEREP_SIZE) {
        cofs = cofs - MQTT_TIMEREP_SIZE;
    }
    if (cofs + len >= MQTT_TIMEREP_SIZE) {
        int first = MQTT_TIMEREP_SIZE - cofs;
        MQTT_FROM_EDS(cofs, data, first);
        MQTT_FROM_EDS(0, data + first, len - first);
    }
    else {
        MQTT_FROM_EDS(cofs, data, len);
    }
    yLeaveCriticalSection(&CbCtx.mqtt_fifo_cs);
}

void mqtt_push_unplug(yStrRef serialref)
{
    if (CbCtx.state == CB_MQTT_ANNOUNCE || CbCtx.state == CB_MQTT_CONNECTED) {
        unplugRecord rec = { 0, 0xff, { serialref, YSTRREF_EMPTY_STRING } };
        ypGetDevAttributesByYdx(wpGetDevYdx(serialref), NULL, &rec.child.nameref);
        mqtt_push_data((u8*)&rec, 6);
    }
}

s32 mqtt_push_not_item(u8 devYdx, u8 funYdx, u8 type, s32 avgval, const u8* data, int len)
{
    int i;
    u32 poww;
    u8 byteVal = 0;
    s32 value = 0;
    poww = 1;
    for (i = 0; i < len; i++) {
        byteVal = data[i];
        value += poww * byteVal;
        poww = poww << 8;
    }
    if (type == 0) {
        //avg = real value
        if ((byteVal & 0x80) != 0) {
            value = value - poww;
        }
    }
    else {
        funYdx |= type;
        if (type == 0x10) {
            //min = value to subtract of avg
            value = avgval - value;
        }
        else {
            //max = value to subtract of avg
            value = avgval + value;
        }
    }
    //dbglog("add not %x:%x val=%d len=%d\n", devYdx, funYdx, value, len);
    mqtt_push_data(&devYdx, 1);
    mqtt_push_data(&funYdx, 1);
    mqtt_push_data((u8*)&value, 4);
    return value;
}

void mqtt_push_not(u8 devydx, const USB_Report_Pkt_V2* report)
{
    int len;
    u8 funYdx = report->funYdx;
    const u8* data;
    len = report->extraLen + 1;
    data = report->data;
    if (len <= 4) {
        // immediate report
        mqtt_push_not_item(devydx, funYdx, 0, 0, data, len);
    }
    else {
        // averaged report: avg,avg-min,max-avg
        s32 avgval;
        u8 detail = *data++;
        len = 1 + (detail & 3);
        avgval = mqtt_push_not_item(devydx, funYdx, 0, 0, data, len);
        data += len;
        len = 1 + ((detail >> 2) & 3);
        mqtt_push_not_item(devydx, funYdx, 0x10, avgval, data, len);
        data += len;
        len = 1 + ((detail >> 4) & 3);
        mqtt_push_not_item(devydx, funYdx, 0x20, avgval, data, len);
    }
}

// devYdx in [0...NB_MAX_DEVICES[                => flag a value change callback
// devYdx in [NB_MAX_DEVICES...2*NB_MAX_DEVICES] => flag an api refresh
// funMask is 0..14  for a regular function
//            0xf    for "module"
//            -1     for all functions
void  mqtt_setFlag(u16 devydx, s16 funYdx)
{
    if (CbCtx.state < CB_MQTT_ANNOUNCE || CbCtx.state > CB_MQTT_CONNECTED) {
        return;
    }
    if (devydx < 2 * NB_MAX_DEVICES) {
        u16 funMask;
        if (funYdx >= 0) {
            funMask = 1 << funYdx;
        }
        else {
            u8 trueDevYdx = (devydx >= NB_MAX_DEVICES ? devydx - NB_MAX_DEVICES : devydx);
            u16 nbfun = ypFunctionCount(trueDevYdx);
            funMask = (u16)((1 << nbfun) - 1);
            if (devydx >= NB_MAX_DEVICES) {
                funMask |= 0x8000;
            }
        }
        cbBuff.wbuff[devydx] |= funMask;
    }
}

static u16 mqtt_build_publish(yHash serial, yHash devname, yHash funcId, yHash funcName)
{
    u16 slen, len, k;

    // publish pkt header
    CbCtx.buf[0] = (char)((MQTT_PKT_PUBLISH << 4) + (CbCtx.mqtt_ctx.noRetain ? 0 : 1));
    // size is patched later
    // add topic
    k = 4;
    if (CbCtx.mqtt_ctx.rootTopicLen) {
        // add topic root if cbURL contain root=XXX
        memcpy(CbCtx.buf + k, CbCtx.mqtt_ctx.rootTopic, CbCtx.mqtt_ctx.rootTopicLen);
        k += CbCtx.mqtt_ctx.rootTopicLen;
        CbCtx.buf[k++] = '/';
    }
    if (devname != INVALID_HASH_IDX && devname != YSTRREF_EMPTY_STRING) {
        yHashGetStr(devname, CbCtx.buf + k, HASH_BUF_SIZE);
    }
    else {
        yHashGetStr(serial, CbCtx.buf + k, HASH_BUF_SIZE);
    }
    CbCtx.buf[2] = 0;
    slen = ystrlen(CbCtx.buf + 4) & 0xff;
    CbCtx.buf[3] = (u8)slen;
    len = slen + 4;
    if (funcId != INVALID_HASH_IDX) {
        CbCtx.buf[len++] = '/';
        if (funcName != INVALID_HASH_IDX && funcName != YSTRREF_EMPTY_STRING) {
            yHashGetStr(funcName, CbCtx.buf + len, HASH_BUF_SIZE);
        }
        else {
            yHashGetStr(funcId, CbCtx.buf + len, HASH_BUF_SIZE);
        }
        slen = ystrlen(CbCtx.buf + len) & 0xff;
        CbCtx.buf[3] += slen + 1;
        len += slen;
    }
    return len;
}

u16 yDumpState;
static u16 mqtt_build_publish_rdy(yStrRef serialRef, yStrRef nameRef, char value)
{
    u16 len = mqtt_build_publish(serialRef, nameRef, YSTRREF_mODULE_STRING, INVALID_HASH_IDX);
    memcpy(CbCtx.buf + len, "/rdy", 4);
    len += 4;
    CbCtx.buf[len++] = value;
    CbCtx.buf[3] += 4;
    CbCtx.buf[1] = len - 2;
    return len;
}

static int mqtt_open_template(char fixedFile, int testOnly)
{
    yFarFifoBuf* fifo;
    yStrRef serial, funcId;
    u8      baseclass;
    int     rc;
    char*   className, * p;
    char    errmsg[256];
    char    tmpbuff[HASH_BUF_SIZE + 6];

    if (ypGetAttributesByYdx((u8)CbCtx.mqtt_ctx.devYdx, (u8)CbCtx.mqtt_ctx.funYdx, &serial, NULL, &funcId, NULL, &baseclass, NULL, NULL) < 0) {
        // Problem accessing function, skip it
        return -1;
    }
    ystrcpy(CbCtx.buf, sizeof(CbCtx.buf), "GET /`MQTT/y");
    p = CbCtx.buf + ystrlen(CbCtx.buf);
    className = p;
    if (fixedFile) {
        *p++ = fixedFile;
    } else {
        yHashGetStr(funcId, p, YOCTO_FUNCTION_LEN + 1);
        *p &= ~0x20; // capitalize first letter
        while (*p) p++;
        while (p[-1] < 'A') p--;
    }
    ystrcpy(p, 5, ".fmt");
    // first try with category-specific suffix
    rc = yoctohubQuickGet(CbCtx.buf + 6, NULL, errmsg);
    if (baseclass == YOCTO_AKA_YSENSOR && rc < 0 && !fixedFile) {
        // try to load default sensor template
        ystrcpy(className, sizeof(CbCtx.buf) - 12, "Sensor");
        ystrcat(className, sizeof(CbCtx.buf) - 12, ".fmt");
        rc = yoctohubQuickGet(CbCtx.buf + 6, NULL, errmsg);
    }
    if (rc < 0) {
        // Cannot load advertising template, skip function
        return -1;
    }
    if (testOnly) {
        return 1;
    }
    if (LocalhostOpen(&CbCtx.apiskt, 0, &CbCtx.errmsg) < 0) {
        return 0;
    }
    CbCtx.lo_open = 1;
    CbCtx.mqtt_ctx.httpBufLen = 0;
    CbCtx.timeref = ytime();
    fifo = LocalhostGetInputFifo(CbCtx.apiskt);
    yPushFarFifo(fifo, (u8*)CbCtx.buf, ystrlen(CbCtx.buf));
    // append substitution arguments
    ystrcpy(tmpbuff, sizeof(tmpbuff), "?sn=");
    yHashGetStr(serial, tmpbuff + 4, sizeof(tmpbuff) - 4);
    yPushFarFifo(fifo, (u8*)tmpbuff, ystrlen(tmpbuff));
    ystrcpy(tmpbuff, sizeof(tmpbuff), "&fn=");
    yHashGetStr(funcId, tmpbuff + 4, sizeof(tmpbuff) - 4);
    yPushFarFifo(fifo, (u8*)tmpbuff, ystrlen(tmpbuff));
    if (!fixedFile) {
        ystrcpy(tmpbuff, sizeof(tmpbuff), "&pa=");
        yu16toa(CbCtx.mqtt_ctx.recIdx - 1, tmpbuff + 4, sizeof(tmpbuff) - 4);
        yPushFarFifo(fifo, (u8*)tmpbuff, ystrlen(tmpbuff));
    }
    if (CbCtx.mqtt_ctx.rootTopicLen) {
        yPushFarFifo(fifo, (u8*)"&root=", 6);
        yPushFarFifo(fifo, (u8*)CbCtx.mqtt_ctx.rootTopic, CbCtx.mqtt_ctx.rootTopicLen);
    }
    yPushFarFifo(fifo, (u8*)" \r\n\r\n", 5);

    return 1;
}

static int mqtt_open_devfunydx(void)
{
    yFarFifoBuf* fifo;
    yStrRef serial, funcId;
    int     slen;
    char    tmpbuff[HASH_BUF_SIZE + 2];

    if (ypGetAttributesByYdx((u8)CbCtx.mqtt_ctx.devYdx, (u8)CbCtx.mqtt_ctx.funYdx, &serial, NULL, &funcId, NULL, NULL, NULL, NULL) < 0) {
        return -1; // unknown function
    }
    // Request localhost access
    if (LocalhostOpen(&CbCtx.apiskt, 0, &CbCtx.errmsg) < 0) {
        return 0;
    }
    CbCtx.lo_open = 1;
    CbCtx.mqtt_ctx.httpBufLen = 0;
    CbCtx.timeref = ytime();
    fifo = LocalhostGetInputFifo(CbCtx.apiskt);
    if (serial == SerialRef) {
        yPushFarFifo(fifo, (u8*)"GET /`", 5);
    }
    else {
        yPushFarFifo(fifo, (u8*)"GET /bySerial/", 14);
        yHashGetStr(serial, tmpbuff, HASH_BUF_SIZE);
        slen = ystrlen(tmpbuff);
        tmpbuff[slen++] = '/';
        yPushFarFifo(fifo, (u8*)tmpbuff, slen);
    }
    yPushFarFifo(fifo, (u8*)"api/", 4);
    yHashGetStr(funcId, tmpbuff, HASH_BUF_SIZE);
    slen = ystrlen(tmpbuff);
    yPushFarFifo(fifo, (u8*)tmpbuff, slen);
    yPushFarFifo(fifo, (u8*)".json", 5);

    return 1;
}

static void mqtt_config_to_utf8(void)
{
    u16 k, len = 0;
    for (k = 0; k < CbCtx.mqtt_ctx.httpBufLen; k++) {
        u8 c = cbBuff.buff[MQTT_HTTPBUF_START + k];
        if (c > 127) {
            len++;
        }
    }
    CbCtx.mqtt_ctx.httpBufLen += len;
    while (len > 0 && k-- > 0) {
        u8 c = cbBuff.buff[MQTT_HTTPBUF_START + k];
        if (c < 127) {
            cbBuff.buff[MQTT_HTTPBUF_START + k + len] = c;
        }
        else {
            cbBuff.buff[MQTT_HTTPBUF_START + k + len] = (c & 0x3f) + 0x80;
            cbBuff.buff[MQTT_HTTPBUF_START + k + --len] = (c > 0xbf ? 0xc3 : 0xc2);
        }
    }
}

void mqtt_trigger_announce(void)
{
    if (CbCtx.state == CB_MQTT_CONNECTED) {
        CbCtx.mqtt_ctx.nextAnnounce = ytime() + 3000;
    }
}

#define CbConnIsInvalid()             (CbCtx.strym == INVALID_SOCKET_MULTI)
#define CbConnSetInvalid()            do{ CbCtx.strym = INVALID_SOCKET_MULTI; }while(0)
#define CbIsConnected()               yTcpCheckSocketStillValidMulti(CbCtx.strym,ytcpLastErrMsg)
#define CbFlush()

static char ytcpLastErrMsg[512];
static u8 inputFifoBuf[512];
yFifoBuf cbInputFifo;

static void CbProcessInput(void)
{
    struct timeval timeout;
    fd_set  fds;
    int     sktmax;
    int     avail;

    avail = yFifoGetFree(&cbInputFifo);
    if (!avail) return;
    memset(&timeout, 0, sizeof(timeout));
    FD_ZERO(&fds);
    sktmax = (int)yTcpFdSetMulti(CbCtx.strym, &fds, 0);
    if (select(sktmax + 1, &fds, NULL, NULL, &timeout) > 0) {
        u8 buff[256];
        if (avail > sizeof(buff)) {
            avail = sizeof(buff);
        }
        avail = yTcpReadMulti(CbCtx.strym, buff, avail, ytcpLastErrMsg);
        if (avail > 0) {
            yPushFifo(&cbInputFifo, buff, avail);
        }
    }
}

static u16  CbIsGetReady()
{
    CbProcessInput();
    return yFifoGetUsed(&cbInputFifo);
}

static void CbPeekArray(u8* Data, u16 Len, u16 ahead)
{
    CbProcessInput();
    yPeekFifo(&cbInputFifo, Data, Len, ahead);
}

static void CbGetArray(u8* Data, u16 Len)
{
    CbProcessInput();
    yPopFifo(&cbInputFifo, Data, Len);
}

static u16  CbIsPutReady()
{
    fd_set  fds;
    int     sktmax;

    struct timeval timeout;
    memset(&timeout, 0, sizeof(timeout));
    FD_ZERO(&fds);
    sktmax = (int)yTcpFdSetMulti(CbCtx.strym, &fds, 0);
    if (select(sktmax + 1, NULL, &fds, NULL, &timeout) > (YSOCKET)0) {
        return 1500;
    }
    return 0;
}

static void CbPutArray(const u8* Data, u16 Len)
{
    *ytcpLastErrMsg = 0;
    yTcpWriteMulti(CbCtx.strym, Data, Len, ytcpLastErrMsg);
}

static void CbClose(void)
{
    yTcpCloseMulti(CbCtx.strym);
    yFifoEmpty(&cbInputFifo);
}

void dectoa(s32 value, char* buff, int bufflen, int scale)
{
    char* p;

#ifdef WINDOWS_API
    sprintf_s(buff, bufflen, "%.3f", value * 0.001);
#else
    sprintf(buff, "%.3f", value * 0.001);
#endif
    for (p = buff; *p && *p != '.'; p++);
    if (!*p) return;
    while (p[1]) p++;
    while (p > buff && p[-1] == '0') p--;
    if (p[-1] == '.') p--;
    if (p == buff) *p++ = '0';
    *p = 0;
}

// Initialize MQTT callback data structures
void mqtt_init(void)
{
    memset(&CbCtx, 0, sizeof(CbCtx));
    cbBuff.len = 0;
    CbConnSetInvalid();
    yInitializeCriticalSection(&CbCtx.mqtt_fifo_cs);
    yFifoInit(&cbInputFifo, inputFifoBuf, sizeof(inputFifoBuf));
}

// Perform some work on mqtt reports
// return 1 if some work has been done, 0 otherwise
int mqtt_run(void)
{
    CBState initState = CbCtx.state;
    u16     i, j, k, len, tmp_len;
    char*   p, tmpbuff[32];
    int     res = 0, err;
    yFarFifoBuf* fifo;

    // Handle disconnection from callback
    if (CbCtx.state > CB_IDLE && CbCtx.state < CB_FAILED) {
        if (!isHubConnected(&CbCtx.errmsg)) {
            res = 1;
            if (CbCtx.lo_open) {
                LocalhostDisconnect(CbCtx.apiskt);
                CbCtx.lo_open = 0;
            }
            if (!CbConnIsInvalid()) {
                CbClose();
                CbConnSetInvalid();
            }
            CbCtx.timeref = ytime();
            CbCtx.state = CB_IDLE;
        }
    }
    switch (CbCtx.state) {
    case CB_FAILED:
        if (CbCtx.errmsg) {
            if (CbCtx.buf[0]) {
                ylogf("MQTT: %s [%s]\n", CbCtx.errmsg, CbCtx.buf);
            } else {
                ylogf("MQTT: %s\n", CbCtx.errmsg);
            }
        }
        if (!CbConnIsInvalid()) {
            CbClose();
            CbConnSetInvalid();
        }
        if (CbCtx.lo_open) {
            // cleanup api socket for next call
            LocalhostDisconnect(CbCtx.apiskt);
            CbCtx.lo_open = 0;
        }
        CbCtx.timeref = ytime();
        CbCtx.state = CB_IDLE;
        ylogf("Disconnected from MQTT broker\n");
        break;
    case CB_IDLE:
        // Wait 3 seconds before retrying reconnection to MQTT broker
        if (ytime() - CbCtx.timeref < 3000) break;
        // Wait until the remote hub is connected
        if (!isHubConnected(&CbCtx.errmsg)) break;
        // Wait until we know the hub logical name (we need it for "last will" subscribe)
        if (NetworkHubNameRef == INVALID_HASH_IDX) break;
        ylogf("Connected to Yoctopuce hub, about to connect to MQTT broker\n");
        CbCtx.buf[0] = 0;
        CbCtx.errmsg = CbCtx.buf + 1;
        CbCtx.timeref = ytime();
        err = yTcpOpenMulti(&CbCtx.strym, Globalp.mqttbroker_host, Globalp.mqttbroker_port, Globalp.flags & MQTT_USE_SSL, MQTT_CONNECT_TIMEOUT * 1000, CbCtx.errmsg);
        if (err < 0) {
            CbCtx.state = CB_FAILED;
            break;
        }
        CbCtx.errmsg = NULL;
        CbCtx.state = CB_MQTT_TRY;
        break;
    case CB_MQTT_TRY:
        // Setup MQTT options
        CbCtx.mqtt_ctx.rootTopic = "";
        CbCtx.mqtt_ctx.rootTopicLen = 0;
        CbCtx.mqtt_ctx.flags = 0;
        if (Globalp.flags & MQTT_RW_MODE) {
            CbCtx.mqtt_ctx.rwMode = 1;
        }
        if (Globalp.flags & MQTT_NO_INSTANT_VALUE) {
            CbCtx.mqtt_ctx.disableIv = 1;
        }
        if (Globalp.flags & MQTT_NO_RETAIN) {
            CbCtx.mqtt_ctx.noRetain = 1;
        }
        if (Globalp.rootTopic) {
            CbCtx.mqtt_ctx.rootTopic = Globalp.rootTopic;
            CbCtx.mqtt_ctx.rootTopicLen = (u16)strlen(Globalp.rootTopic);
            if (CbCtx.mqtt_ctx.rootTopicLen > 0 && CbCtx.mqtt_ctx.rootTopic[CbCtx.mqtt_ctx.rootTopicLen - 1] == '/') {
                // remove trailing slash
                CbCtx.mqtt_ctx.rootTopicLen--;
            }
        }
        if (Globalp.clientID) {
            p = (char *)Globalp.clientID;
        } else {
            p = "MqttBridge";
        }
        i = ystrlen(p);
        // Build MQTT connect packet
        len = mqtt_build_publish_rdy(SerialRef, NetworkHubNameRef, '0') - 3;
        // convert "publish topic" to "last will topic"
        memmove(CbCtx.buf + 14 + i, CbCtx.buf + 2, len);
        CbCtx.buf[0] = (char)(MQTT_PKT_CONNECT << 4);
        CbCtx.buf[1] = 12;
        // Protocol name and level (OASIS MQTT 3.1.1)
        CbCtx.buf[2] = 0;
        CbCtx.buf[3] = 4;
        CbCtx.buf[4] = 'M';
        CbCtx.buf[5] = 'Q';
        CbCtx.buf[6] = 'T';
        CbCtx.buf[7] = 'T';
        CbCtx.buf[8] = 4;   // Protocol Level
        // Connection flags
        CbCtx.buf[9] = 6;   // Will + Clean Session
        // Keep alive (60s)
        CbCtx.buf[10] = 0;
        CbCtx.buf[11] = 60;
        CbCtx.buf[12] = 0;
        CbCtx.buf[13] = i & 0x7f;
        memcpy(CbCtx.buf + 14, p, i);
        i += 14 + len;
        // Will value is not formatted the same as a publish
        CbCtx.buf[i++] = 0;
        CbCtx.buf[i++] = 1;
        CbCtx.buf[i++] = '0';
        // append username, if any
        if (Globalp.mqttUser) {
            int len = ystrlen(Globalp.mqttUser);
            CbCtx.buf[i++] = 0;
            CbCtx.buf[i++] = len;
            memcpy(CbCtx.buf + i, Globalp.mqttUser, len);
            i += len;
            // update flags
            CbCtx.buf[9] |= 1 << 6;
        }
        // append password, if any
        if (Globalp.mqttPass) {
            int len = ystrlen(Globalp.mqttPass);
            CbCtx.buf[i++] = 0;
            CbCtx.buf[i++] = len;
            memcpy(CbCtx.buf + i, Globalp.mqttPass, len);
            i += len;
            // update flags
            CbCtx.buf[9] |= 1 << 7;
        }
        // patch len with correct size
        CbCtx.buf[1] = i - 2;
        // Write CONNECT packet
        CbPutArray((u8*)CbCtx.buf, i);
        CbFlush();
        CbCtx.state = CB_MQTT_WAIT_CONNECT;
        CbCtx.timeref = ytime();
        CbCtx.contentLength = 2;
        break;
    case CB_MQTT_WAIT_CONNECT:
        len = CbIsGetReady();
        if (len < CbCtx.contentLength) {
            if (ytime() - CbCtx.timeref > CALLBACK_TIMEOUT) {
                CbCtx.errmsg = YFSTR(NET_68, "timeout reading header");
                CbCtx.buf[0] = 0;
                CbCtx.state = CB_FAILED;
            }
            break;
        }
        CbPeekArray((u8*)tmpbuff, 2, 0);
        CbCtx.contentLength = 2 + tmpbuff[1];
        if (len < CbCtx.contentLength) {
            break;
        }
        if (len > sizeof(tmpbuff)) {
            len = sizeof(tmpbuff);
        }
        CbGetArray((u8*)tmpbuff, len);
        if (tmpbuff[0] != (char)(MQTT_PKT_CONNACK << 4) || tmpbuff[1] > 2 || tmpbuff[3] != 0) {
            if (tmpbuff[3] == 5 || tmpbuff[3] == 4) {
                CbCtx.errmsg = YFSTR(NET_58, "incorrect user/password");
            }
            else {
                CbCtx.errmsg = YFSTR(NET_107, "MQTT?");
            }
            CbCtx.buf[0] = 0;
            CbCtx.state = CB_FAILED;
            break;
        }
        ylogf("Successfully connected to MQTT broker\n");
        CbCtx.writepos = 0;
        // clear bitmaps
        for (i = 0; i < 2 * NB_MAX_DEVICES; i++) {
            cbBuff.wbuff[i] = 0;
        }
        CbCtx.mqtt_ctx.tail_ofs = 0;
        CbCtx.mqtt_ctx.head_ofs = 0;
        CbCtx.mqtt_ctx.devYdx = 0;
        CbCtx.mqtt_ctx.funYdx = 0;
        CbCtx.mqtt_ctx.recIdx = 0;
        CbCtx.mqtt_ctx.subState = 0;
        CbCtx.writepos = 0;
        CbCtx.state = CB_MQTT_ANNOUNCE;
        // if r/w access is not desired, switch to next state
        if (!CbCtx.mqtt_ctx.rwMode) {
            break;
        }
        // subscribe to SET requests
        CbCtx.buf[0] = (char)((MQTT_PKT_SUBSCRIBE << 4) + 2); // QoS is always 1
        // remaining length is patched later
        CbCtx.buf[2] = 0; // message ID = 1
        CbCtx.buf[3] = 1;
        // add topic
        k = 6;
        if (CbCtx.mqtt_ctx.rootTopicLen) {
            // add topic root if cbURL contain root=XXX
            memcpy(CbCtx.buf + k, CbCtx.mqtt_ctx.rootTopic, CbCtx.mqtt_ctx.rootTopicLen);
            k += CbCtx.mqtt_ctx.rootTopicLen;
            CbCtx.buf[k++] = '/';
        }
        memcpy(CbCtx.buf + k, "+/+/set/#", 10); // terminal NUL is QoS (0)
        k += 10;
        CbCtx.buf[4] = 0;
        CbCtx.buf[5] = (k - 7) & 0xff;
        // update pktlen and send
        CbCtx.buf[1] = (k - 2) & 0xff;
        CbPutArray((u8*)CbCtx.buf, k);
        break;
    case CB_MQTT_ANNOUNCE:
#if 0
        ylogf("Announce %u/%u/%u st=%u lo=%u\n", CbCtx.mqtt_ctx.devYdx, CbCtx.mqtt_ctx.funYdx,
            CbCtx.mqtt_ctx.recIdx, CbCtx.mqtt_ctx.subState, CbCtx.lo_open & 1);
#endif
        if (CbCtx.mqtt_ctx.subState == 0 && CbCtx.lo_open == 0) {
            // Prepare to announce next function
            s16 openRes;
            while (CbCtx.mqtt_ctx.devYdx < NB_MAX_DEVICES) {
                k = ypFunctionCount((u8)CbCtx.mqtt_ctx.devYdx);
                if (k > 0) {
                    if (CbCtx.mqtt_ctx.funYdx == k) {
                        CbCtx.mqtt_ctx.funYdx = 0xf; // special for "module"
                    }
                    if (CbCtx.mqtt_ctx.funYdx < k || CbCtx.mqtt_ctx.funYdx == 0xf) {
                        break;
                    }
                    // prepare to publish value and API as well for this device
                    mqtt_setFlag(CbCtx.mqtt_ctx.devYdx, -1);
                    mqtt_setFlag(NB_MAX_DEVICES + CbCtx.mqtt_ctx.devYdx, -1);
                }
                CbCtx.mqtt_ctx.devYdx++;
                CbCtx.mqtt_ctx.funYdx = 0;
                CbCtx.mqtt_ctx.recIdx = 0;
            }
            if (CbCtx.mqtt_ctx.devYdx >= NB_MAX_DEVICES) {
                // repeat announce in 5 minutes
                CbCtx.mqtt_ctx.nextAnnounce = ytime() + 300000;
                CbCtx.state = CB_MQTT_CONNECTED;
                break;
            }
            if (!CbCtx.mqtt_ctx.recIdx) {
                // make sure there is a template for this function
                openRes = mqtt_open_template(0, 1);
                if (openRes < 0) {
                    // Problem accessing function, skip it
                    CbCtx.mqtt_ctx.funYdx++;
                    CbCtx.mqtt_ctx.recIdx = 0;
                    break;
                }
                // Pre-read the footer for this function to determine its length
                openRes = mqtt_open_template('Z', 0);
            }
            else {
                openRes = mqtt_open_template(0, 0);
            }
            if (openRes < 0) {
                // Problem accessing function, skip it
                CbCtx.mqtt_ctx.funYdx++;
                CbCtx.mqtt_ctx.recIdx = 0;
                CbCtx.mqtt_ctx.subState = 0;
                break;
            }
            if (openRes == 0) {
                break;
            }
            CbCtx.mqtt_ctx.subState = 1;
            res = 1;
        }
        else if (CbCtx.mqtt_ctx.subState == 1) {
            // Read configuration record describing footer (recIdx 0) or yp entry (recIdx 1+)
            fifo = LocalhostGetOutputFifo(CbCtx.apiskt);
            if (EdsLoadFromFifo(fifo, (u8*)CbCtx.buf, sizeof(CbCtx.buf), MQTT_HTTPBUF_START, MQTT_HTTPBUF_SIZE, (u16*)&CbCtx.mqtt_ctx.httpBufLen,
                YFSTR(NET_541, "failed to load callback template")) < 0) {
                break;
            }
            if (LocalhostIsDone(CbCtx.apiskt) && CbIsPutReady() >= 5) {
                LocalhostDisconnect(CbCtx.apiskt);
                CbCtx.lo_open = 0;
                res = 1;
                // if configuration buffer is empty (or almost), skip to next function
                if (CbCtx.mqtt_ctx.httpBufLen < 5) {
                    CbCtx.mqtt_ctx.funYdx++;
                    CbCtx.mqtt_ctx.recIdx = 0;
                    CbCtx.mqtt_ctx.subState = 0;
                    break;
                }
                // configuration buffer fully loaded, but we need to convert it to UTF-8
                mqtt_config_to_utf8();
                if (!CbCtx.mqtt_ctx.recIdx) {
                    // this was just a try run to determine footer length
                    CbCtx.mqtt_ctx.tailBufLen = CbCtx.mqtt_ctx.httpBufLen;
                    CbCtx.mqtt_ctx.recIdx = 1;
                    CbCtx.mqtt_ctx.subState = 0;
                    break;
                }
                // prepare to publish discovery topic
                len = CbCtx.mqtt_ctx.httpBufLen + CbCtx.mqtt_ctx.tailBufLen;
                tmpbuff[0] = (char)((MQTT_PKT_PUBLISH << 4) + (CbCtx.mqtt_ctx.noRetain ? 0 : 1));
                tmpbuff[1] = (u8)len;
                if (CbCtx.mqtt_ctx.httpBufLen > 127) {
                    tmpbuff[1] |= 0x80;
                    tmpbuff[2] = (u8)(len >> 7);
                    len = 3;
                }
                else {
                    len = 2;
                }
                // search for the buffer for the \r\n marker (end of topic name)
                for (k = 1; k < CbCtx.mqtt_ctx.httpBufLen; k++) {
                    char c = cbBuff.buff[MQTT_HTTPBUF_START + k];
                    if (c == '\r' || c == '\n') break;
                }
                // add topic length in tmpbuff
                tmpbuff[len++] = 0;
                tmpbuff[len++] = k & 0xff;
                CbPutArray((u8*)tmpbuff, len);
                // drop CRLF at end of topic in cbBuff
                while (k-- > 0) {
                    char c = cbBuff.buff[MQTT_HTTPBUF_START + k];
                    cbBuff.buff[MQTT_HTTPBUF_START + k + 2] = c;
                }
                // skip first two characters, topic length was sent via tmpbuff
                CbCtx.mqtt_ctx.httpBufPos = 2;
                CbCtx.mqtt_ctx.subState = 2;
            }
        }
        else if (CbCtx.mqtt_ctx.subState == 2 || CbCtx.mqtt_ctx.subState == 5) {
            if (CbCtx.mqtt_ctx.httpBufPos < CbCtx.mqtt_ctx.httpBufLen) {
                while ((len = CbIsPutReady()) > 0) {
                    if (len > sizeof(tmpbuff)) {
                        len = sizeof(tmpbuff);
                    }
                    if (len > CbCtx.mqtt_ctx.httpBufLen - CbCtx.mqtt_ctx.httpBufPos) {
                        len = CbCtx.mqtt_ctx.httpBufLen - CbCtx.mqtt_ctx.httpBufPos;
                    }
                    if (!len) break;
                    EdsMoveFrom(MQTT_HTTPBUF_START + CbCtx.mqtt_ctx.httpBufPos, (u8*)tmpbuff, len);
                    CbPutArray((u8*)tmpbuff, len);
                    CbCtx.mqtt_ctx.httpBufPos += len;
                    res = 1;
                }
            }
            else {
                if (CbCtx.mqtt_ctx.subState == 2) {
                    CbCtx.mqtt_ctx.subState = 3;
                }
                else { // substate == 5
                 // move on to next record
                    CbCtx.mqtt_ctx.recIdx++;
                    CbCtx.mqtt_ctx.subState = 0;
                }
            }
        }
        else if (CbCtx.mqtt_ctx.subState == 3 && CbCtx.lo_open == 0) {
            // append the record footer record
            int openRes = mqtt_open_template('Z', 0);
            if (openRes < 0) {
                // Problem accessing footer ? skip record
                CbCtx.mqtt_ctx.recIdx++;
                CbCtx.mqtt_ctx.subState = 0;
                break;
            }
            if (openRes == 0) {
                break;
            }
            CbCtx.mqtt_ctx.subState = 4;
            res = 1;
        }
        else if (CbCtx.mqtt_ctx.subState == 4) {
            // Read footer
            fifo = LocalhostGetOutputFifo(CbCtx.apiskt);
            if (EdsLoadFromFifo(fifo, (u8*)CbCtx.buf, sizeof(CbCtx.buf), MQTT_HTTPBUF_START, MQTT_HTTPBUF_SIZE, (u16*)&CbCtx.mqtt_ctx.httpBufLen,
                YFSTR(NET_541, "failed to load callback template")) < 0) {
                break;
            }
            if (LocalhostIsDone(CbCtx.apiskt) && CbIsPutReady() >= 5) {
                LocalhostDisconnect(CbCtx.apiskt);
                CbCtx.lo_open = 0;
                // footer fully loaded, but we need to convert it to UTF-8
                mqtt_config_to_utf8();
                CbCtx.mqtt_ctx.httpBufPos = 0;
                CbCtx.mqtt_ctx.subState = 5;
                res = 1;
            }
        }
        break;
    case CB_MQTT_CONNECTED:
        CbCtx.mqtt_ctx.mqttIdle = 0;
        if (CbCtx.lo_open == 0 && CbCtx.mqtt_ctx.httpBufLen < 0) {
            if ((s32)(ytime() - CbCtx.mqtt_ctx.nextAnnounce) > 0) {
                // time to announce
                CbCtx.mqtt_ctx.devYdx = 0;
                CbCtx.mqtt_ctx.funYdx = 0;
                CbCtx.mqtt_ctx.recIdx = 0;
                CbCtx.mqtt_ctx.subState = 0;
                CbCtx.state = CB_MQTT_ANNOUNCE;
            }
            else {
                yStrRef serial, funcId, funcName, devname;
                char    funcVal[YOCTO_PUBVAL_LEN];
                int     slen, openRes;
                Notification_funydx funInfo;

                // Publish timed reports values and unplug events
                u32 fifo_len = mqtt_fifo_size();
                while (fifo_len >= 6) {
                    s32 value;
                    u8 devYdx = mqtt_peek_byte(0);
                    u8 funYdx = mqtt_peek_byte(1);
                    u8 baseClass;
                    if (devYdx == 0 && funYdx == 0xff) {
                        // special case: device unplug event, mark device as not available
                        unplugData unplugDev;
                        mqtt_peek((u8*)&unplugDev, 2, 4);
                        len = mqtt_build_publish_rdy(unplugDev.serialref, unplugDev.nameref, '0');
                    }
                    else if (ypGetAttributesByYdx(devYdx, funYdx & 0xf, &serial, &devname, &funcId, &funcName, &baseClass, NULL, NULL) >= 0) {
                        // setup publish pkt header
                        len = mqtt_build_publish(serial, devname, funcId, funcName);
                        int type = funYdx >> 4;
                        if (type == 1) {
                            ystrcpy(CbCtx.buf + len, sizeof(CbCtx.buf) - len, "/min");
                        }
                        else if (type == 2) {
                            ystrcpy(CbCtx.buf + len, sizeof(CbCtx.buf) - len, "/max");
                        }
                        else {
                            ystrcpy(CbCtx.buf + len, sizeof(CbCtx.buf) - len, "/avg");
                        }
                        CbCtx.buf[3] += 4;
                        len += 4;
                        // add payload
                        mqtt_peek((u8*)&value, 2, 4);
                        //dbglog("send not %x:%x val=%d len=%d\n", devYdx, funYdx, value, fifo_len);
                        dectoa(value, CbCtx.buf + len, 16, 1);
                        slen = ystrlen(CbCtx.buf + len) & 0xff;
                        len += slen;
                        // update pktlen
                        CbCtx.buf[1] = len - 2;
                    }
                    else {
                        len = 0;
                    }
                    if (len > 0) {
                        // publish value
                        if (CbIsPutReady() < len) {
                            CbFlush();
                            return 0;
                        }
                        CbPutArray((u8*)CbCtx.buf, len);
                        // reset keep-alive watchdog
                        CbCtx.timeref = ytime();
                    }
                    yEnterCriticalSection(&CbCtx.mqtt_fifo_cs);
                    fifo_len = mqtt_drop();
                    yLeaveCriticalSection(&CbCtx.mqtt_fifo_cs);
                }

                // Publish advertised values
                for (i = 0; i < NB_MAX_DEVICES; i++) {
                    __eds__ u16* funflags = cbBuff.wbuff + CbCtx.writepos;
                    u16 funflags_val = *funflags;
                    if (funflags_val) {
                        for (j = 0; j < 15; j++) {
                            u16 mask = 1 << j;
                            if (funflags_val & mask) {
                                u8 baseClass;
                                if (ypGetAttributesByYdx((u8)CbCtx.writepos, (u8)j, &serial, &devname, &funcId, &funcName, &baseClass, &funInfo, CbCtx.buf) < 0) break;
                                if (CbCtx.mqtt_ctx.disableIv && baseClass == YOCTO_AKA_YSENSOR) break;
                                decodePubVal(funInfo, CbCtx.buf, funcVal);
                                if (funcVal[0] != 0) {
                                    // setup publish pkt header
                                    len = mqtt_build_publish(serial, devname, funcId, funcName);
                                    // add payload
                                    slen = ystrlen(funcVal);
                                    memcpy(CbCtx.buf + len, funcVal, slen);
                                    len += slen;
                                    // update pktlen
                                    CbCtx.buf[1] = len - 2;
                                    // send
                                    k = CbIsPutReady();
                                    if (k < len) {
                                        if (ytime() - CbCtx.timeref > CALLBACK_TIMEOUT) {
                                            CbCtx.errmsg = YFSTR(NET_51, "timeout writing body");
                                            CbCtx.buf[0] = 0;
                                            CbCtx.state = CB_FAILED;
                                        }
                                        CbFlush();
                                        return 0;
                                    }
                                    CbPutArray((u8*)CbCtx.buf, len);
                                    // reset keep-alive watchdog
                                    CbCtx.timeref = ytime();
                                }
                                *funflags ^= mask;
                            }
                        }
                    }
                    CbCtx.writepos++;
                    if (CbCtx.writepos == NB_MAX_DEVICES) {
                        CbCtx.writepos = 0;
                    }
                }

                // Publish api topic when requested
                for (i = 0; i < NB_MAX_DEVICES; i++) {
                    __eds__ u16* funflags = cbBuff.wbuff + NB_MAX_DEVICES + CbCtx.writepos;
                    u16 funflags_val = *funflags;
                    if (funflags_val) {
                        CbCtx.mqtt_ctx.devYdx = CbCtx.writepos;
                        for (j = 0; j <= 15; j++) {
                            u16 mask = 1 << j;
                            if (funflags_val & mask) {
                                CbCtx.mqtt_ctx.funYdx = j;
                                openRes = mqtt_open_devfunydx();
                                if (openRes < 1) {
                                    // unknown function? skip it
                                    *funflags ^= mask;
                                    continue;
                                }
                                if (openRes == 0) {
                                    // localhost not available, retry later
                                    return 0;
                                }
                                *funflags ^= mask;
                                // submit request as-is to localhost, will be published when available
                                fifo = LocalhostGetInputFifo(CbCtx.apiskt);
                                yPushFarFifo(fifo, (u8*)" \r\n\r\n", 5);
                                return 0;
                            }
                        }
                    }
                    CbCtx.writepos++;
                    if (CbCtx.writepos == NB_MAX_DEVICES) {
                        CbCtx.writepos = 0;
                    }
                }

                // Make sure we always send a packet within keep-alive period
                if ((u16)(ytime() - CbCtx.timeref) >= 60000u) {
                    tmpbuff[0] = (char)(MQTT_PKT_PINGREQ << 4);
                    tmpbuff[1] = 0;
                    CbPutArray((u8*)tmpbuff, 2);
                    CbFlush();
                    // reset keep-alive watchdog
                    CbCtx.timeref = ytime();
                }

                // handle incoming messages
                i = CbIsGetReady();
                if (i == 0) {
                    // we have processed all outgoing messages, and there is no pending message
                    CbCtx.mqtt_ctx.mqttIdle = 1;
                }
                if (i < 2) {
                    break;
                }
                if (i > sizeof(CbCtx.buf)) {
                    i = sizeof(CbCtx.buf);
                }
                CbPeekArray((u8*)CbCtx.buf, i, 0);
                len = (u8)CbCtx.buf[1];
                if (len & 0x80) {
                    if (i < 3) break;
                    len = (len & 0x7f) + (CbCtx.buf[2] << 7);
                    if (len > sizeof(CbCtx.buf) - 3) {
                        // unsupported big packet (out of sync?), close connection
                        CbCtx.errmsg = YFSTR(NET_107, "MQTT?");
                        CbCtx.buf[0] = 0;
                        CbCtx.state = CB_FAILED;
                        CbGetArray(NULL, i);
                        return 0;
                    }
                }
                len += 2;
                if (i < len) break;
                // ylogf("MQTT msg: %02x (len=%d)\n", (u8)CbCtx.buf[0], len);
                if (((u8)CbCtx.buf[0] >> 4) != MQTT_PKT_PUBLISH || !CbCtx.mqtt_ctx.rwMode) {
                dropMsg:
                    // drop message silently, we don't need to handle it
                    CbGetArray(NULL, len);
                    break;
                }
                // Make sure the message is for one of our devices
                tmp_len = ((u16)(u8)CbCtx.buf[2] << 8) + ((u8)CbCtx.buf[3]);
                if (tmp_len < CbCtx.mqtt_ctx.rootTopicLen + 10 || tmp_len > CbCtx.mqtt_ctx.rootTopicLen + sizeof(CbCtx.buf)) {
                    goto dropMsg; // not for us
                }
                tmp_len = tmp_len + 4; // end of topic
                j = CbCtx.mqtt_ctx.rootTopicLen + 5; // start of device identifier
                for (i = j; i < tmp_len && CbCtx.buf[i] != '/'; i++);
                serial = yHashTestBuf((u8*)CbCtx.buf + j, i - j);
                if (serial == INVALID_HASH_IDX || serial == YSTRREF_EMPTY_STRING || ++i >= tmp_len) {
                    goto dropMsg; // not for us
                }
                CbCtx.mqtt_ctx.devYdx = wpGetDevYdx(serial);
                if (CbCtx.mqtt_ctx.devYdx < 0) {
                    YAPI_DEVICE dev = wpSearchByNameHash(serial);
                    if (dev < 0) goto dropMsg; // not for us
                    serial = (u16)dev;
                    CbCtx.mqtt_ctx.devYdx = wpGetDevYdx(serial);
                    if (CbCtx.mqtt_ctx.devYdx < 0) {
                        goto dropMsg; // not for us
                    }
                }
                j = i;
                while (i < tmp_len && CbCtx.buf[i] != '/') i++;
                funcId = yHashTestBuf((u8*)CbCtx.buf + j, i - j);
                i += 4; // skip over "/set"
                if (funcId == INVALID_HASH_IDX || funcId == YSTRREF_EMPTY_STRING || i >= tmp_len) {
                    goto dropMsg; // not for us
                }
                if (funcId == YSTRREF_mODULE_STRING) {
                    CbCtx.mqtt_ctx.funYdx = 15;
                }
                else {
                    CbCtx.mqtt_ctx.funYdx = ypSearchByDevYdx((u8)CbCtx.mqtt_ctx.devYdx, funcId);
                    if (CbCtx.mqtt_ctx.funYdx < 0) {
                        goto dropMsg; // unknown function
                    }
                }
                openRes = mqtt_open_devfunydx();
                if (openRes < 0) {
                    goto dropMsg; // unknown function
                }
                if (openRes == 0) {
                    break; // localhost not available, retry later
                }
                fifo = LocalhostGetInputFifo(CbCtx.apiskt);
                CbCtx.buf[i] = '?';         // replace '/' by '?' before attribute name
                yPushFarFifo(fifo, (u8*)CbCtx.buf + i, tmp_len - i);
                tmpbuff[0] = '=';
                yPushFarFifo(fifo, (u8*)tmpbuff, 1);
                yPushFarFifo(fifo, (u8*)CbCtx.buf + tmp_len, len - tmp_len);
                yPushFarFifo(fifo, (u8*)" \r\n\r\n", 5);
                // Consume processed message
                CbGetArray(NULL, len);
                // clear api refresh for this function, it it was pending
                cbBuff.wbuff[NB_MAX_DEVICES + CbCtx.mqtt_ctx.devYdx] &= ~(1 << CbCtx.mqtt_ctx.funYdx);
            }
            CbFlush();
        }
        else if (CbCtx.lo_open) {
            yStrRef serial, devname, funcId, funcName;

            // Read output of previous command (an api.json)
            fifo = LocalhostGetOutputFifo(CbCtx.apiskt);
            if (EdsLoadFromFifo(fifo, (u8*)CbCtx.buf, sizeof(CbCtx.buf), MQTT_HTTPBUF_START, MQTT_HTTPBUF_SIZE, (u16*)&CbCtx.mqtt_ctx.httpBufLen,
                YFSTR(NET_59, "request failed")) < 0) {
                break;
            }
            // prepare to publish header in CbCtx.buf
            len = 0;
            funcName = YSTRREF_EMPTY_STRING;
            if (ypGetAttributesByYdx((u8)CbCtx.mqtt_ctx.devYdx, (u8)CbCtx.mqtt_ctx.funYdx, &serial, &devname, &funcId, &funcName, NULL, NULL, NULL) >= 0) {
                len = mqtt_build_publish(serial, devname, funcId, funcName);
                if (CbIsPutReady() < len + 5) break;
            }
            // When we are ready to output topic and all output is complete, proceed
            if (LocalhostIsDone(CbCtx.apiskt)) {
                LocalhostDisconnect(CbCtx.apiskt);
                CbCtx.lo_open = 0;
                if (!len) {
                    // failed to build topic, do not publish
                    CbCtx.mqtt_ctx.httpBufLen = -1;
                    break;
                }
                // append /api to topic
                ystrcpy(CbCtx.buf + len, sizeof(CbCtx.buf) - len, "/api");
                CbCtx.buf[3] += 4;
                len += 4;
                // api json payload fully loaded, but we need to convert it to UTF-8 before publish
                mqtt_config_to_utf8();
                k = len - 2 + CbCtx.mqtt_ctx.httpBufLen;
                if (k <= 0x7f) {
                    CbCtx.buf[1] = (u8)k;
                }
                else {
                    // need to insert an extra length byte...
                    memmove(CbCtx.buf + 3, CbCtx.buf + 2, len - 2);
                    CbCtx.buf[1] = (u8)(k | 0x80);
                    CbCtx.buf[2] = (u8)(k >> 7);
                    len++;
                }
                // send publish header
                CbPutArray((u8*)CbCtx.buf, len);
                CbCtx.mqtt_ctx.httpBufPos = 0;
            }
        }
        else if (CbCtx.mqtt_ctx.httpBufPos < CbCtx.mqtt_ctx.httpBufLen) {
            while ((len = CbIsPutReady()) > 0) {
                if (len > sizeof(tmpbuff)) {
                    len = sizeof(tmpbuff);
                }
                if (len > CbCtx.mqtt_ctx.httpBufLen - CbCtx.mqtt_ctx.httpBufPos) {
                    len = CbCtx.mqtt_ctx.httpBufLen - CbCtx.mqtt_ctx.httpBufPos;
                }
                if (!len) break;
                EdsMoveFrom(MQTT_HTTPBUF_START + CbCtx.mqtt_ctx.httpBufPos, (u8*)tmpbuff, len);
                CbPutArray((u8*)tmpbuff, len);
                CbCtx.mqtt_ctx.httpBufPos += len;
            }
        }
        else {
            yStrRef serial, devname;
            if (ypGetDevAttributesByYdx((u8)CbCtx.mqtt_ctx.devYdx, &serial, &devname) >= 0) {
                len = mqtt_build_publish_rdy(serial, devname, '1');
                if (CbIsPutReady() < len) break;
                CbPutArray((u8*)CbCtx.buf, len);
            }
            CbCtx.mqtt_ctx.httpBufLen = -1;
        }
        break;
    }

    if (initState != CbCtx.state) {
#ifdef DEBUG_CB_STATE_CHANGES
        dbglog("CBState %s -> %s\n", cb_state_str[initState], cb_state_str[CbCtx.state]);
#endif
        res = 1;
    }
    return res;
}
