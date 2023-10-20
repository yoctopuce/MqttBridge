/*********************************************************************
 *
 * $Id: winStartup.c 57313 2023-10-20 09:00:00Z seb $
 *
 * Windows Entry point for the service/deamon
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

/*********************************************************************************

    WINDOWS SPECIFIC STARTUP CODE 

**********************************************************************************/

#define __FILENAME__   "winStartup"
#include "MqttBridge.h"
#include "windows.h"

#ifdef WINDOWS_API

#include <tchar.h>
#include <iphlpapi.h>

//#define DEBUG_SERVICE_START
#ifdef DEBUG_SERVICE_START

void srvlog(char *msg)
{   
    FILE *file; 
    if(YFOPEN(&file,"c:\\tmp\\srvstart.txt","a")!=0){
        return;
    }
    fwrite(msg,1,strlen(msg),file);
    fclose(file);
}

#endif

#include <windows.h> 
#include <stdio.h> 
 
BOOL CtrlHandler( DWORD fdwCtrlType ) 
{ 
  switch( fdwCtrlType ) 
  { 
    // Handle the CTRL-C signal. 
    case CTRL_C_EVENT: 
	  CtrlCHandler(fdwCtrlType);
      return( TRUE );
 
    // CTRL-CLOSE: confirm that the user wants to exit. 
    case CTRL_CLOSE_EVENT: 
	  CtrlCHandler(fdwCtrlType);
      return( TRUE ); 
 
    // Pass other signals to the next handler. 
    case CTRL_BREAK_EVENT: 
      //Beep( 900, 200 ); 
      //printf( "Ctrl-Break event\n\n" );
      return FALSE; 
 
    case CTRL_LOGOFF_EVENT: 
      //Beep( 1000, 200 ); 
      //printf( "Ctrl-Logoff event\n\n" );
      return FALSE; 
 
    case CTRL_SHUTDOWN_EVENT: 
      //Beep( 750, 500 ); 
      //printf( "Ctrl-Shutdown event\n\n" );
      return FALSE; 
 
    default: 
      return FALSE; 
  } 
} 
 
#define SVCNAME TEXT("yMqttBridge")
#define SVCDESCR "MqttBridge is Yoctopuce tool to connect any Yoctopuce hub available on the network to a MQTT broker"

void  WinError(const char *msg, DWORD err)
{
    LPTSTR lpMsgBuf;
    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        err,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0, NULL );
    fprintf(stderr, "%s (%d:%s)",msg,err,lpMsgBuf);
}

// -----------------------------------------------------------------
// main windows entrypoint that will be used by app and service
// -----------------------------------------------------------------

int winmain(int argc, char * argv[])
{
    // Initialize Winsock 2.2
    char  errmsg[YOCTO_ERRMSG_LEN*2];
    WSADATA wsaData;
    int     iResult;
#ifdef DEBUG_SERVICE_START
    srvlog("winmain:start\n");
#endif
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0){
        WinError("Unable to start Windows Socket",iResult);
#ifdef DEBUG_SERVICE_START
        srvlog("Unable to start Windows Socket\n");
#endif
        return -1;  
    }
    
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE );

    if(TunnelMain(errmsg)){
#ifdef DEBUG_SERVICE_START
        srvlog("ERROR: ");   
        srvlog(errmsg);
        srvlog("\n");
#endif
    }
#ifdef DEBUG_SERVICE_START
    srvlog("winmain:stop\n");
#endif
    WSACleanup();
    return 0; 
}

// -----------------------------------------------------------------
// Variables that are needed ffor the hiddend window
// -----------------------------------------------------------------
HANDLE    hthread; // thead of the hidden windows
HINSTANCE hinst;    
HWND      hwind=NULL;
HANDLE    hnotify;

// -----------------------------------------------------------------
// Service functions
// -----------------------------------------------------------------

SERVICE_STATUS          ServiceStatus; 
SERVICE_STATUS_HANDLE   ServiceStatusHandle; 
HANDLE                  ServiceStopEvent = NULL;

void ReportServiceStatus( u32 CurrentState, u32 Win32ExitCode, u32 WaitHint);
void ServiceControlhandler(DWORD ctrl,DWORD eventType,LPVOID eventData,LPVOID contex);
VOID SvcReportEvent(LPTSTR szFunction);

int  InstallService(int install, const char *exe, const char* args, char* errrmsg)
{
    SC_HANDLE           SCman,service;
    TCHAR               path[MAX_PATH];
    SERVICE_DESCRIPTION sd;
    
    if( !GetModuleFileNameA( NULL, path, MAX_PATH )){
        sprintf_s(errrmsg, YOCTO_ERRMSG_LEN, "Cannot install service (%d)\n", GetLastError());
        return YAPI_IO_ERROR;
    }

    if(strlen(args)+strlen(path)>MAX_PATH){
        sprintf_s(errrmsg, YOCTO_ERRMSG_LEN, "Too many arguments for services please contact support@yoctopuce.com\n");
        return YAPI_IO_ERROR;
    }
    strcat_s(path,MAX_PATH,args);
  
#ifdef DEBUG_SERVICE_START
    srvlog(path);
    srvlog("\n");
#endif
    SCman=OpenSCManager(NULL,NULL,SC_MANAGER_CREATE_SERVICE);
    if(!SCman) {
        sprintf_s(errrmsg, YOCTO_ERRMSG_LEN, "OpenSCManager failed (%d)\n", GetLastError());
        return YAPI_IO_ERROR;
    }
    //check if exist
    service=OpenService(SCman,SVCNAME,SERVICE_ALL_ACCESS);
    if(!service){
        if(!install){
            sprintf_s(errrmsg, YOCTO_ERRMSG_LEN, "unable to open MqttBridge service\n");
            CloseServiceHandle(SCman);
            return YAPI_IO_ERROR;
        }
        // no -> create it
        service=CreateService(
					SCman,
					SVCNAME,
					_T("Yoctopuce MqttBridge"),
                    SERVICE_ALL_ACCESS,
					SERVICE_WIN32_OWN_PROCESS,
                    SERVICE_AUTO_START,
					SERVICE_ERROR_NORMAL,
                    path, NULL,NULL,NULL,NULL,NULL);
        if (!service) {
        	sprintf_s(errrmsg, YOCTO_ERRMSG_LEN, "CreateService failed (%d)\n", GetLastError());
        	CloseServiceHandle(SCman);
            return YAPI_IO_ERROR;
        }
           // Change the service description.

        sd.lpDescription = SVCDESCR;
        if( !ChangeServiceConfig2(
            service,                 // handle to service
            SERVICE_CONFIG_DESCRIPTION, // change: description
            &sd) )                      // new description
        {
            printf("Unable to set service description (%d)\n",GetLastError());
        }
        printf("Service successfully installed\n");
    } else{
        if(!install){
            SERVICE_STATUS dummy;
            ControlService(service,SERVICE_CONTROL_STOP,&dummy);
            if(DeleteService(service)){
                 printf("Service successfully uninstalled\n");
            }
            return YAPI_SUCCESS;
        }
        printf("Service allready installed\n");
    }

    StartService(service,0,0);
    CloseServiceHandle(service);
    CloseServiceHandle(SCman);
    return YAPI_SUCCESS;
}

//
// Purpose: 
//   Entry point for the service
//
// Parameters:
//   dwArgc - Number of arguments in the lpszArgv array
//   lpszArgv - Array of strings. The first string is the name of
//     the service and subsequent strings are passed by the process
//     that called the StartService function to start the service.
// 
// Return value:
//   None.
//
void WINAPI SvcMain( DWORD dwArgc, LPTSTR  *lpszArgv )
{
    
#ifdef DEBUG_SERVICE_START
    srvlog("Service Starting\n");
#endif
    // Register the handler function for the service    
    ServiceStatusHandle = RegisterServiceCtrlHandlerEx(
							SVCNAME,
							(LPHANDLER_FUNCTION_EX) ServiceControlhandler,0);
    if(!ServiceStatusHandle ){ 
        WinError("Unable to register Service handler",GetLastError());
        SvcReportEvent(TEXT("RegisterServiceCtrlHandler")); 
        return; 
    } 
#ifdef DEBUG_SERVICE_START
    srvlog("RegisterServiceCtrlHandlerEx done\n");
#endif
    // These SERVICE_STATUS members remain as set here
    ServiceStatus.dwServiceType = SERVICE_WIN32_OWN_PROCESS; 
    ServiceStatus.dwServiceSpecificExitCode = 0;    

    // Report initial status to the SCM

    ReportServiceStatus( SERVICE_START_PENDING, NO_ERROR, 3000 );
#ifdef DEBUG_SERVICE_START
    srvlog("ReportServiceStatus done\n");
#endif
    // TO_DO: Declare and set any required variables.
    //   Be sure to periodically call ReportSvcStatus() with 
    //   SERVICE_START_PENDING. If initialization fails, call
    //   ReportSvcStatus with SERVICE_STOPPED.

#ifdef DEBUG_SERVICE_START
    {
        DWORD i;
        srvlog("SRVmain\n");
        srvlog(__DATE__);
        srvlog(" ");
        srvlog(__TIME__);
        srvlog("\n");

        for(i=0;i<dwArgc;i++){
            srvlog("arg:");
            srvlog(lpszArgv[i]);
            srvlog("\n");
        }
    }
#endif

    //ParseArguments(dwArgc,lpszArgv);
#ifdef DEBUG_SERVICE_START
    srvlog("ParseArguments done\n");
#endif
    // Report running status when initialization is complete.
    ReportServiceStatus( SERVICE_RUNNING, NO_ERROR, 0 );
#ifdef DEBUG_SERVICE_START
    srvlog("ReportServiceStatus done\n");
#endif
    winmain(dwArgc, lpszArgv);
#ifdef DEBUG_SERVICE_START
    srvlog("winmain done\n");
#endif
    ReportServiceStatus( SERVICE_STOPPED, NO_ERROR, 0 );
    return;
}

void ReportServiceStatus( u32 CurrentState, u32 Win32ExitCode, u32 WaitHint)
{
    static DWORD dwCheckPoint = 1;

    // Fill in the SERVICE_STATUS structure.

    ServiceStatus.dwCurrentState = CurrentState;
    ServiceStatus.dwWin32ExitCode = Win32ExitCode;
    ServiceStatus.dwWaitHint = WaitHint;

    if (CurrentState == SERVICE_START_PENDING)
        ServiceStatus.dwControlsAccepted = 0;
    else 
        ServiceStatus.dwControlsAccepted = SERVICE_ACCEPT_STOP;

    if ( (CurrentState == SERVICE_RUNNING) || (CurrentState == SERVICE_STOPPED) )
        ServiceStatus.dwCheckPoint = 0;
    else 
        ServiceStatus.dwCheckPoint = dwCheckPoint++;

    // Report the status of the service to the SCM.
    SetServiceStatus(ServiceStatusHandle,&ServiceStatus);
}

//
// Purpose: 
//   Called by SCM whenever a control code is sent to the service
//   using the ControlService function.
//
// Parameters:
//   dwCtrl - control code
// 
// Return value:
//   None
//
void  ServiceControlhandler(DWORD ctrl, DWORD eventType, LPVOID eventData, LPVOID contex)
{
    // Handle the requested control code. 
    switch(ctrl) {  
    case SERVICE_CONTROL_STOP:
        ReportServiceStatus(SERVICE_STOP_PENDING, NO_ERROR, 0);
        EnsureDaemonExit();
        break;
    case SERVICE_CONTROL_INTERROGATE: 
         break;
    case SERVICE_CONTROL_DEVICEEVENT:
#if 0
        ylog("received device event notification\n");
        if (eventType == DBT_DEVICEARRIVAL){
            ylog("Message DBT_DEVICEARRIVAL\n");
            if(IsOurDevice((LPARAM)eventData)){
                ylog("plug notifyed\n");
                break;
            }      
        }else if (eventType == DBT_DEVICEREMOVECOMPLETE){     
            //ylog("Message DBT_DEVICEREMOVECOMPLETE\n");   
            if(IsOurDevice((LPARAM)eventData)){
                ylog("unplug notifyed\n");
                break;
            }
        }
#endif
        break;
    default: 
         break;
   } 
    ReportServiceStatus(ServiceStatus.dwCurrentState, NO_ERROR, 0);
        
}

//
// Purpose: 
//   Logs messages to the event log
//
// Parameters:
//   szFunction - name of function that failed
// 
// Return value:
//   None
//
// Remarks:
//   The service must have an entry in the Application event log.
//
VOID SvcReportEvent(LPTSTR szFunction) 
{ 
    HANDLE hEventSource;
    LPCTSTR lpszStrings[2];
    TCHAR Buffer[80];

    hEventSource = RegisterEventSource(NULL, SVCNAME);

    if( NULL != hEventSource )
    {
        sprintf_s(Buffer, 80, "%s failed with %d", szFunction, GetLastError());

        lpszStrings[0] = SVCNAME;
        lpszStrings[1] = Buffer;

        ReportEvent(hEventSource,        // event log handle
                    EVENTLOG_ERROR_TYPE, // event type
                    0,                   // event category
                    0,                   // event identifier
                    NULL,                // no security identifier
                    2,                   // size of lpszStrings array
                    0,                   // no binary data
                    lpszStrings,         // array of strings
                    NULL);               // no binary data

        DeregisterEventSource(hEventSource);
    }
}

// -----------------------------------------------------------------
// plug/unplug detection stuff
// -----------------------------------------------------------------
//Globally Unique Identifier (GUID) for HID class devices.  Windows uses GUIDs to identify things.
GUID InterfaceClassGuid = {0x4d1e55b2, 0xf16f, 0x11cf, 0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30};

int main (int argc, char * argv[])
{

    SERVICE_TABLE_ENTRY DispatchTable[] = { 
        { SVCNAME, (LPSERVICE_MAIN_FUNCTION) SvcMain }, 
        { NULL, NULL } 
    }; 



	{
		char value[128];
		DWORD value_length = 128;
		LONG  res;
		HKEY hKey;
		 
		// open registry key
		res = RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("SYSTEM\\CurrentControlSet\\Control"), 0, KEY_READ, &hKey);
	    if(res == ERROR_SUCCESS) {
			res = RegQueryValueEx(hKey, TEXT("WaitToKillServiceTimeout"), NULL, NULL, (LPBYTE)value, &value_length);
			if(res == ERROR_SUCCESS) {
				u32 winval = atoi(value);
				if(winval < EnsureDaemonExitDelay){
					EnsureDaemonExitDelay = winval;
				}
			}
	        RegCloseKey(hKey);
	    }
	}

#ifdef DEBUG_SERVICE_START
    int i;
    srvlog("main\n");
    srvlog(__DATE__);
    srvlog(" ");
    srvlog(__TIME__);
    srvlog("\n");

    for(i=0;i<argc;i++){
        srvlog("arg:");
        srvlog(argv[i]);
        srvlog("\n");
    }
#endif

    ParseArguments(argc,argv);
#ifdef DEBUG_SERVICE_START
    srvlog("main: argparsed\n");
#endif
    if (Globalp.flags & VHUB_RUN_IN_DEAMON){ 
        // Service Mode
        // This call returns when the service has stopped. 
        // The process should simply terminate when the call returns.
#ifdef DEBUG_SERVICE_START
        srvlog("main: start service\n");
#endif
        if(!StartServiceCtrlDispatcher( DispatchTable )){
            u32 error = GetLastError();
            WinError("Unable to Start the service",error);            
    #ifdef DEBUG_SERVICE_START
            srvlog("main: Unable to Start the service\n");
    #endif        
            return 1;
        }
    } else{
#ifdef DEBUG_SERVICE_START
        srvlog("main: start std\n");
#endif
#if 0
        if(yCreateDetachedThread(&hthread,HiddenWindowThread,NULL)<0){
            return -1;
        }
#endif
        winmain(argc, argv);      
    } 
#ifdef DEBUG_SERVICE_START
    srvlog("main: exit\n");
#endif
    return 0; 
}
#endif
