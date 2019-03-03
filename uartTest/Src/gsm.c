/*
 * gsm.c
 *
 *  Created on: 4 окт. 2018 г.
 *      Author: shestakov.aa
 */

#include "gsm.h"
#include <stdint.h>
#include <stddef.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Clock.h>
#include "uart.h"
//#include <ti/sysbios/knl/Queue.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "TIMI.h"
#include "MyQueue.h"
#include "parserSMS.h"
#include <stdio.h>
#include <math.h>
#include "QueueAT.h"
#include "parserExecute.h"
#include "flash.h"
#include <xdc/runtime/Error.h>
#include "definePar.h"
#include "Signal.h"

#define TASK_STACK_SIZE_GSM    712
#define TASK_PRIORITY_GSM      4
#define REPIT_COUNT            3

#define COUNT_OPROS            150
#define MAX_SMS_DELTA          5

#define ERROR_GPRS             1

#define TIMEOUT_GPRS           6000//5мин таймаут переподключения
#define TIMEOUT_MAIN           1200// 0.5 мин таймоут после инита, для оиска к gprs
#define TIMEOUT_COMAND         300
#define TIMEOUT_ASK            500

static Task_Struct task_gsm;    /* not static so you can see in ROV */
static Task_Params taskParams_gsm;
static uint8_t taskStack_gsm[TASK_STACK_SIZE_GSM];
StateSendGsm stateSendGsm = FIND;

Error_Block eb_gsm;
Task_Handle gsmTask;

Code curCode;

//StatrSendSMS stateSendSMS = CLEARSMS;

StateGSM stateGSM = XZSTATE;
const char okmess[] = "OK";
const char ermess[] = "ERROR";
const char execute[] = "EXECUTE";
const char gpsg[]="GPSG#";
char paramCode[6];
const char ask[] = "ACK";
int countInit=0;


ComGsm comGsmTab[] =
{
 {AT,  "at\r\r\n", 5 , TIMEOUT_COMAND,3,ER_INIT},
 {IPR, "at+ipr=115200\r\n", 15, TIMEOUT_COMAND,1,ER_INIT },
 {CPIN, "at+cpin?\r\n", 10, TIMEOUT_COMAND, 1,ER_INIT},
 {CMGF, "at+cmgf=1\r\n", 11,TIMEOUT_COMAND,1 ,ER_INIT},
 {CSCS, "at+cscs=\"GSM\"\r\n", 15, TIMEOUT_COMAND,1,ER_INIT },
 {COPS, "at+cops=3,2\r\n", 13, TIMEOUT_COMAND,1 ,ER_INIT},
 {COPS_A, "at+cops?\r\n", 10, TIMEOUT_COMAND,1 ,ER_INIT},
 {CNMI, "at+cnmi=0\r\n", 11, TIMEOUT_COMAND,1 ,ER_INIT},
 {CPMS, "at+cpms=\"ME\",\"ME\",\"ME\"\r\n", 24, TIMEOUT_COMAND,1,ER_INIT },
 {CSQ, "at+csq\r\n", 8, TIMEOUT_COMAND,1 ,ER_INIT},
 {ATE0, "ATE0\r\n", 6, TIMEOUT_COMAND,1 ,ER_INIT},
 {CREG, "at+creg=2\r\n", 11, TIMEOUT_COMAND,1,ER_INIT },
 {CREG_A, "at+creg?\r\n", 10, TIMEOUT_COMAND,1 ,ER},
 {CPMS_A, "at+cpms?\r\n", 10, TIMEOUT_COMAND,1 ,ER},
 {CMGS, "at+cmgs=\"+79117638226\"\r", 23, TIMEOUT_COMAND,1 ,ER_SENDSMS},
 {CMGR, "at+cmgr=000\r\n", 13, TIMEOUT_COMAND,1,ER_READSMS },
 {CMGD, "at+cmgd=0,2\r\n", 13, TIMEOUT_COMAND,1,ER_READSMS },
 {CTRL_Z, "\x1A", 1, TIMEOUT_COMAND,1 ,ER_SENDSMS},
 {UPSD0, "at+upsd=0,0,0\r\n", 15, TIMEOUT_COMAND,1,ER_GPRS  },
 {UPSD1, "at+upsd=0,1,\"internet\"\r\n", 24, TIMEOUT_COMAND,1,ER_GPRS  },
 {UPSD2, "at+upsd=0,2,\"gdata\"\r\n", 21, TIMEOUT_COMAND,1,ER_GPRS  },
 {UPSD3, "at+upsd=0,3,\"gdata\"\r\n", 21, TIMEOUT_COMAND,1,ER_GPRS  },
 {UPSDA3, "at+upsda=0,3\r\n", 14, TIMEOUT_COMAND,1,ER_CON_GPRS },//активация профиля 1
 {UPSDA4, "at+upsda=0,4\r\n", 14, TIMEOUT_COMAND,1,ER_DISCON_GPRS },//деактивация профиля
 {USORD, "at+usord=0,100\r\n", 16, TIMEOUT_COMAND,1,ER_RD_GPRS },// запрос ip адреса
 {USOCR, "at+usocr=6\r\n", 12, TIMEOUT_COMAND,1,ER_CON_GPRS },// открытие сокета 2
 {USOCL, "at+usocl=0\r\n", 12, TIMEOUT_COMAND,1,ER_DISCON_GPRS },//закрытие сокета
 {USOWR, "at+usowr=0,147\r\n", 16, TIMEOUT_COMAND,1,ER_WR_GPRS },//запись данных 4
 {USOWROK, "at+usowr=0,24,\"imei=00000000000&rmc=OK \"\r\n", 42, TIMEOUT_COMAND,1,ER_WROK_GPRS},
 {USOCO, "at+usoco=0,\"84.204.102.210\",6009\r\n", 34, TIMEOUT_COMAND,1,ER_CON_GPRS },//подключение сокета 3
};



//колбэк функция для таска gsm , опрос состояниф gsm
void gsmFnx(UArg arg0, UArg arg1)
{
    while(1){

        UpdateGsmState();
        Task_sleep((50000 / Clock_tickPeriod));

    }
}


void UpdateComand(void){
    PastComU(CMGS, config.server);
    PastComU(UPSD1, config.apn);
    PastComU(UPSD2, config.login);
    PastComU(UPSD3, config.pas);
    PastComU(USOCO, config.ip);
    PastComU(USOWROK, config.id);
}


void  PastComU(TypeCom typeCom, char *past)
{
    int i =0;
    for ( i = 0; i<sizeof(comGsmTab)/sizeof(ComGsm); i++)
    {
        if (typeCom == comGsmTab[i].type){

            break;
        }
    }

    if (typeCom == CMGS){
        //"at+cmgs=\"+79117638226\"\r"
        char* zx =strchr(past, '*');
        strncpy(comGsmTab[i].comand+10, past, (zx-past+1));

        zx = strchr(comGsmTab[i].comand, '*');
        strncpy(zx, "\"\r", 2);
        zx = strchr(comGsmTab[i].comand, '\r');
        int leit = comGsmTab[i].comand+37-(zx+1);
        memset(zx+1, 0x00, leit);
    }
    if (typeCom == UPSD1){
        //"at+upsd=0,1,\"internet\"\r\n"
        char* zx =strchr(past, '*');
        strncpy(comGsmTab[i].comand+13, past, (zx-past+1));
        zx = strchr(comGsmTab[i].comand, '*');
        strncpy(zx, "\"\r\n", 3);
        zx = strchr(comGsmTab[i].comand, '\n');
        int leit = comGsmTab[i].comand+37-(zx+1);
        memset(zx+1, 0x00, leit);
    }
    if (typeCom == UPSD2){
        //"at+upsd=0,2,\"gdata\"\r\n"
        char* zx =strchr(past, '*');
        strncpy(comGsmTab[i].comand+13, past, (zx-past+1));
        zx = strchr(comGsmTab[i].comand, '*');
        strncpy(zx, "\"\r\n", 3);
        zx = strchr(comGsmTab[i].comand, '\n');
        int leit = comGsmTab[i].comand+37-(zx+1);
        memset(zx+1, 0x00, leit);
    }
    if (typeCom == UPSD3){
        //"at+upsd=0,3,\"gdata\"\r\n"
        char* zx =strchr(past, '*');
        strncpy(comGsmTab[i].comand+13, past, (zx-past+1));
        zx = strchr(comGsmTab[i].comand, '*');
        strncpy(zx, "\"\r\n", 3);
        zx = strchr(comGsmTab[i].comand, '\n');
        int leit = comGsmTab[i].comand+37-(zx+1);
        memset(zx+1, 0x00, leit);
    }

    if (typeCom == USOCO){
        //"at+usoco=0,\"84.204.102.210\",6009\r\n"
        char* zx =strchr(past, '*');
        strncpy(comGsmTab[i].comand+12, past, (zx-past+1));
        zx = strchr(comGsmTab[i].comand, '*');
        strncpy(zx, "\",", 2);
        char* zx2 = strchr(config.port, '*');
        zx =strchr(zx, ',');
        strncpy(zx+1, config.port, (zx2-config.port+1));
        zx= strchr(comGsmTab[i].comand, '*');
        strncpy(zx, "\r\n", 2);
        zx = strchr(comGsmTab[i].comand, '\n');
        int leit = comGsmTab[i].comand+37-(zx+1);
        memset(zx+1, 0x00, leit);
    }
    if (typeCom ==USOWROK){
        //"at+usowr=0,24,\"imei=00000000000&rmc=OK\0\"\r\n"
        char* token;
        if (token = strstr(comGsmTab[i].comand, "imei=")){
            strncpy(token+5, past, 11);
        }
        comGsmTab[i].comand[38] = 0x00;
    }


}

//таск инит для gsm
void GsmTaskInit(void){
    InitQueueAt();

    Task_Params_init(&taskParams_gsm);
    taskParams_gsm.stackSize = TASK_STACK_SIZE_GSM;
    taskParams_gsm.priority = TASK_PRIORITY_GSM;
    taskParams_gsm.stack = &taskStack_gsm;
    taskParams_gsm.arg0 = (UInt)1000000;
    Error_init(&eb_gsm);
    System_printf("Init Gsm task!!!\r\n");
    //Task_construct(&task_gsm, gsmFnx, &taskParams_gsm, NULL);
}

//инициализация таска gsm
void GsmInit(void){
    modemParam.stateGSM = INITGSM;
    //CreateParamCode();
    GsmTaskInit();

}


void GsmSetState(bool st){
     if (st){
         Gsm_start();
     }else{
         Gsm_stop();
     }
}

void Gsm_start(void){
    if (gsmTask == NULL){
        modemParam.stateGSM = INITGSM;
        gsmTask = Task_create(gsmFnx,&taskParams_gsm,&eb_gsm);
        if (gsmTask == NULL)
        {
            System_abort("TaskUart creation failed\r\n");
        }
    }

}

void Gsm_stop(void){
    if (gsmTask!=NULL){
        Task_delete(&gsmTask);
    }

}

void CreateParamCode(void){
    paramCode[0] = ':';
    paramCode[1] = config.code[0];
    strncpy(paramCode+2, config.code, 4);
}


TypeCom typeComSend;
char buf[3];
//основная функция опроса gsm
void UpdateGsmState(void){
    switch(modemParam.stateGSM){
        case INITGSM:
            System_printf("stateGSM = INITGSM\r\n");
            busySendAt = false;
            modemParam.busyRead = false;
            modemParam.busySend = false;
            modemParam.falseInit = false;
            modemParam.stateInit = false;
            modemParam.stageSend= SENDXZ;
            stateGprs = GPRSSTART;
            modemParam.busyInit =true;
            modemParam.initTimeout = 0;
            modemParam.gsmTimeout = 0;
            modemParam.maxSMS = 0;
            modemParam.countOpros = 0;
            strncpy(modemParam.mmc, "000", 3);
            strncpy(modemParam.mnc, "00", 2);
            strncpy(modemParam.sid, "0000", 4);
            strncpy(modemParam.lac, "0000", 4);
            //modemParam.stateGSM =INITGPRS ;
            AddElementQueueAt(ATE0);
            AddElementQueueAt(AT);
            AddElementQueueAt(IPR);
            AddElementQueueAt(CPIN);
            AddElementQueueAt(CMGF);
            AddElementQueueAt(CSCS);
            AddElementQueueAt(COPS);
            AddElementQueueAt(COPS_A);
            AddElementQueueAt(CNMI);
            AddElementQueueAt(CPMS);
            AddElementQueueAt(CREG);
            AddElementQueueAt(CREG_A);
            AddElementQueueAt(CSQ);
            modemParam.stateGSM =WAITINITGSM ;
            //modemParam.messSMS = SetCodeGprs( 24);
            break;
        case WAITINITGSM:
            System_printf("stateGSM = WAITINITGSM\r\n");
            if (!modemParam.busyInit){
                if (modemParam.falseInit){
                    modemParam.stateGSM = XZSTATE;

                } else {
                    modemParam.stateGSM = WAITGPRS;
                    modemParam.stateInit = true;
                }
            }


            break;

        case OPROSGSM:
            System_printf("stateGSM = OPROSGSM\r\n");

            modemParam.stateGSM = OPROSWAIT;
            break;
        case OPROSWAIT:
            modemParam.countOpros= 0;
            modemParam.busyRead = false;
            AddElementQueueAt(CREG_A);
            AddElementQueueAt(CPMS_A);
            if (modemParam.gprsOn){
                AddElementQueueAt(USORD);
            }
            modemParam.stateGSM = MAINWAIT;
            break;
        case WAITGPRS:
            System_printf("stateGSM = WAITGPRS\r\n");
            modemParam.initTimeout++;

                if (modemParam.initTimeout> TIMEOUT_MAIN){
                    modemParam.initTimeout = 0;
                    modemParam.stateGSM = OPROSWAIT;
                }
                if (modemParam.gprsOn || modemParam.gprsSleep){
                    modemParam.initTimeout = 0;
                    modemParam.stateGSM = OPROSWAIT;
                }


            break;
        case MAINWAIT:

            modemParam.countOpros++;
            if (!isQueueEmpty()&&(!modemParam.busySend)){
                if (!modemParam.gprsConni){
                    modemParam.busySend = true;
                    //modemParam.stateGSM = LOGIKASEND;
                    modemParam.stageSend = SENDINIT;
                    modemParam.countOpros= 0;

                    break;
                }

            }
            if ((modemParam.inputsms>0)&&(!modemParam.busyRead)){
                modemParam.stateGSM = READSMS;
                modemParam.countOpros= 0;
                break;
            }

            if ((modemParam.countOpros>=COUNT_OPROS)){
                modemParam.countOpros= 0;
                modemParam.stateGSM = OPROSGSM;
                break;
            }


            break;


        case GPRSREAD:
            System_printf("stateGSM = GPRSREAD\r\n");
            AddElementQueueAt(USORD);
            modemParam.stateGSM = MAINWAIT;
            break;

        case WAITSENDMESSSMS:
            System_printf("stateGSM = WAITSENDMESSSMS\r\n");
            modemParam.stateGSM = MAINWAIT;
            AddElementQueueAt(CMGS);
            AddElementQueueAt(CTRL_Z);
            break;

        case READSMS:
            System_printf("stateGSM = READSMS\r\n");
            modemParam.busyRead = true;
            modemParam.stateGSM = WAITREADSMS;
            break;

        case WAITREADSMS:
            System_printf("stateGSM = WAITREADSMS\r\n");
            modemParam.stateGSM = MAINWAIT;

            AddElementQueueAt(CMGR);
            AddElementQueueAt(CMGD);
            break;

        case RESETGSM:
            System_printf("stateGSM = RESETGSM\r\n");
            break;
        case GSMTIMEOUT:
            modemParam.countOpros++;
            if (modemParam.countOpros>COUNT_OPROS){
                modemParam.countOpros = 0;
                AddElementQueueAt(CREG_A);
                //AddElementQueueAt(CPMS_A);
                if (!modemParam.gsmOff){
                    modemParam.stateGSM = INITGSM;
                }
            }
            break;
        case XZSTATE:
            System_printf("stateGSM = XZSTATE\r\n");
            break;
    }
    ParserQuick();

    GprsModul();

    SendModul();

        if ((!isQueueEmptyAt())||(busySendAt))
        {
            if (!busySendAt){
                busySendAt = true;
                stateSendGsm = FIND;
                typeComSend = GetElementQueueAt();
                curCom = findCom(typeComSend);
                curCom.comState = NO;
                ClearTxBuf();
                //memset(readSmsBuf, 0x00, sizeof(readSmsBuf));
            }
            else {
                parseAT(txBuf, sizeof(txBuf));
                SendCommGSM(typeComSend);

                if (curCom.comState!=NO){
                    busySendAt = false;
                    if (curCom.comState!=OK){
                        ParserError(curCom.comState);
                    }
                }



            }

        }




}

void ParserQuick(void){
    if (strstr(txBuf, "+UUSOCL:")){
        modemParam.gprsOn = false;
        modemParam.gprsUUCLR = true;
        //stateGprs = GPRSDISCONECT;
        //ClearTxBuf();
    }
    if (strstr(txBuf, "+UUSORD:")){
        //modemParam.gprsOn = false;
        //modemParam.gprsUUCLR = true;
        AddElementQueueAt(USORD);
        ClearTxBuf();
    }
}

void SendModul(void){
    if (modemParam.busySend){
        switch(modemParam.stageSend){
            case SENDINIT:
                curCode = GetElementQueue();
                modemParam.curSignal = GetSignal(curCode.codeId);
                modemParam.stageSend = SENDLOGICK;
                modemParam.timeoutAsk = 0;

                //modemParam.sendFlag = false;
                break;
            case SENDLOGICK:
                modemParam.sendGprsFlag = false;
                modemParam.sendSmsFlag = false;
                if (modemParam.curSignal.chanel == ALL){
                    if (modemParam.gprsSleep){
                        modemParam.stageSend = SENDSMS;
                    }else {
                        modemParam.stageSend = SENDGPRS;
                    }
                }
                if (modemParam.curSignal.chanel == SMS){
                    modemParam.stageSend = SENDSMS;
                }
                if (modemParam.curSignal.chanel == GPRS){
                    if (!modemParam.gprsSleep){
                        modemParam.stageSend = SENDGPRS;
                    }else{
                        modemParam.curSignal.flagsend = true;
                        modemParam.stageSend = SENDEND;
                    }
                }
                break;
            case SENDWAIT:
                break;
            case SENDGPRS:
                modemParam.sendGprsFlag = true;
                modemParam.timeoutAsk = 0;
                if (modemParam.gprsOn){
                    modemParam.stageSend = SENDTIMEOUT;
                    AddElementQueueAt(USOWR);
                }
                if (modemParam.gprsSleep||!modemParam.gprsOn){
                    modemParam.stageSend = SENDERRORGPRS;
                }

                break;
            case SENDWAITGPRS:

                modemParam.stageSend = SENDTIMEOUT;

                break;
            case SENDSMS:
                modemParam.timeoutAsk = 0;
                modemParam.sendSmsFlag = true;
                AddElementQueueAt(CMGS);
                AddElementQueueAt(CTRL_Z);
                modemParam.stageSend = SENDTIMEOUT;
                break;
            case SENDTIMEOUT:
                modemParam.timeoutAsk++;
                if (modemParam.curSignal.flagsend){
                    modemParam.stageSend = SENDEND;
                    curCode.errorSend = false;

                }
                if (modemParam.timeoutAsk>TIMEOUT_ASK){
                    modemParam.stageSend = SENDERRORGPRS;
                    curCode.errorSend = true;
                }
                if (modemParam.sendGprsFlag){
                    if (!modemParam.gprsOn || modemParam.gprsSleep){
                        modemParam.stageSend = SENDGPRS;
                    }
                }

                break;

            case SENDERRORGPRS:
                if (modemParam.curSignal.chanel == ALL){
                    modemParam.stageSend = SENDSMS;
                    modemParam.sendGprsFlag = false;
                }else{
                    modemParam.stageSend = SENDEND;
                }

                break;
            case SENDEND:
                Deleteelement(curCode);
                modemParam.stageSend = SENDXZ;
                modemParam.busySend = false;
                break;
            case SENDXZ:
                break;
            default:
                break;

        }
    }
}

void GprsReConnect(void){
    stateGprs = GPRSDISCONECT;
    modemParam.gprsReconnect = true;
}

void GsmPausa(void){
    modemParam.stateGSM = GSMTIMEOUT;
    modemParam.gsmRoaming = true;
}

void GprsDiscon(void){
    if (!modemParam.gprsSleep){
        stateGprs = GPRSDISCONECT;
        modemParam.gprsDiscon = true;
    }
}

void  GprsModul(void){
    switch (stateGprs) {
        case GPRSSTART:

            if (!modemParam.gsmRoaming){
                System_printf("stateGSM = GPRSSTART\r\n");
                modemParam.gprsOn = false;
                modemParam.gprsInit =false;
                modemParam.gprsInit=false;
                modemParam.gprsFalseInit=false;
                modemParam.gprsSleep = false;
                modemParam.gprsErrorCoount = 0;
                modemParam.gprsTimeOut = 0;
                modemParam.gprsParser = true;
                modemParam.gprsReconnect = false;
                modemParam.gprsConni=true;
                modemParam.gprsDiscon = false;
                stateGprs = GPRSINIT;
            } else {
                System_printf("stateGSM = ROUMING\r\n");
            }
            break;

        case GPRSINIT:
            System_printf("stateGSM = GPRSINIT\r\n");
            if (modemParam.stateInit){
                modemParam.gprsInit = false;
                modemParam.gprsFalseInit = false;
                AddElementQueueAt(UPSD0);
                AddElementQueueAt(UPSD1);
                AddElementQueueAt(UPSD2);
                AddElementQueueAt(UPSD3);
                stateGprs = GPRSWAITINIT;
                modemParam.gprsConni=true;
            }
            break;
        case GPRSWAITINIT:
            System_printf("stateGSM = GPRSWAITINIT\r\n");
            modemParam.countOpros= 0;
            if (modemParam.gprsInit){
                if (modemParam.gprsFalseInit){
                    modemParam.gprsStateInit = false;
                    stateGprs = GPRSDISCONECT;
                }else {
                    modemParam.gprsStateInit = true;
                    stateGprs = GPRSCONNECT;
                    modemParam.gprsErrorCoount = 0;
                }
            }
            break;
        case GPRSCONNECT:
            System_printf("stateGSM = GPRSCONNECT\r\n");
            modemParam.gprsConni=true;
            modemParam.gprsDiscon = false;
            AddElementQueueAt(UPSDA3);
            AddElementQueueAt(USOCR);
            AddElementQueueAt(USOCO);
            modemParam.gprsConInit = false;
            modemParam.gprsConFalse = false;
            stateGprs = GPRSWAITCONNECT;
            break;
        case GPRSWAITCONNECT:
            System_printf("stateGSM = GPRSWAITCONNECT\r\n");
            modemParam.countOpros= 0;
            if (modemParam.gprsConInit){
                if (modemParam.gprsConFalse){
                    stateGprs = GPRSDISCONECT;
                }else {
                    //modemParam.gprsStateInit = true;
                    stateGprs = GPRSMAIN;
                    modemParam.gprsErrorCoount = 0;
                    modemParam.gprsOn = true;
                }
            }
            break;
        case GPRSDISCONECT:
            System_printf("stateGSM = GPRSDISCONECT\r\n");

            modemParam.gprsErrorCoount++;
            modemParam.gprsOn = false;
            AddElementQueueAt(USOCL);
            AddElementQueueAt(UPSDA4);
            modemParam.gprsConni=true;
            if (modemParam.gprsDiscon){
                stateGprs = GPRSTIMEOUT;
                break;
            }
            if (modemParam.gprsReconnect){
                modemParam.gprsReconnect = false;
                modemParam.gprsErrorCoount = 0;
                stateGprs = GPRSSTART;
                break;
            }
            if (modemParam.gprsErrorCoount>ERROR_GPRS){
                stateGprs = GPRSTIMEOUT;
                //modemParam.gprsErrorCoount = 0;
                break;
            }

            if (modemParam.gprsFalseInit){
                stateGprs = GPRSINIT;
                //modemParam.gprsErrorCoount = 0;
                break;
            }
            if (modemParam.gprsConFalse){
                stateGprs = GPRSCONNECT;
                break;
            }
            if (modemParam.gprsUUCLR){
                stateGprs = GPRSCONNECT;
                if (!modemParam.gprsConFalse){
                    modemParam.gprsErrorCoount = 0;
                }

                modemParam.gprsUUCLR = false;
                break;
            }




            break;
        case GPRSTIMEOUT:
            System_printf("stateGSM = GPRSTIMEOUT\r\n");
            modemParam.gprsOn = false;
            modemParam.gprsSleep = true;
            modemParam.gprsConni=false;
            modemParam.gprsTimeOut++;
            if (modemParam.gprsTimeOut > TIMEOUT_GPRS){
                modemParam.gprsSleep = false;
                modemParam.gprsTimeOut = 0;
                stateGprs = GPRSSTART;
            }

            break;
        case GPRSMAIN:
            modemParam.gprsDiscon = false;
            modemParam.gprsConni=false;
            modemParam.gprsErrorCoount = 0;
            if (!modemParam.gprsOn){
                stateGprs = GPRSDISCONECT;
                break;
            }
            break;
        default:
            break;
    }


}



void ParserError(ComState comstateEr){
    //System_printf("Error!!!\r\n");
    if (comstateEr == ER_READSMS){
        System_printf("ER_READSMS!!!\r\n");
        modemParam.errorSend = true;
    }
    if (comstateEr == ER_GPRS){
        System_printf("ER_GPRS!!!\r\n");
        modemParam.gprsFalseInit = true;
    }
    if (comstateEr == ER_INIT){
        System_printf("ER_INIT!!!\r\n");
        modemParam.falseInit = true;
    }
    if (comstateEr == ER_DISCON_GPRS){
        System_printf("ER_DISCON_GPRS!!!\r\n");
        //modemParam.falseInit = true;
    }
    if (comstateEr == ER_CON_GPRS){
        System_printf("ER_CON_GPRS!!!\r\n");
        modemParam.gprsConFalse = true;
        modemParam.gprsConInit = true;
    }
    if (comstateEr == ER_WR_GPRS){
        System_printf("ER_WR_GPRS!!!\r\n");
        modemParam.gprsOn = false;
    }
    if (comstateEr == ER_WROK_GPRS){
        System_printf("ER_WROK_GPRS!!!\r\n");

    }
    if (comstateEr == ER_SENDSMS){
        System_printf("ER_SENDSMS!!!\r\n");
    }
}





void toArray(int number, char *numberArray, int size)
    {

        int i;
        for (i = 0; i<size; i++){
            double del = pow(10.0, (double)(size-i-1));
            numberArray[i] = (int)(number/del) + 0x30;
            number = number - ((int)(number/del))*del;

        }

    }



void CreateComAndSend(ComGsm com){
    if (curCom.type == CMGR){

        toArray(modemParam.inputsms, buf, sizeof(buf));
        memcpy(curCom.comand+8, buf, sizeof(buf));
        //PastComU(CMGR, buf);
        modemParam.inputsms--;
    }

    UartSend(curCom.comand, curCom.size);
}


int countTimeout=0;
int repicCount = 0;

void SendCommGSM(TypeCom typeCom){

    switch (stateSendGsm) {
        case FIND:
            messSend = true;
            //curCom = findCom(typeCom);
            stateSendGsm =SENDUART;
            repicCount = 0;
            break;
        case SENDUART:
            wait_ans_com = true;
            wait_ans_res = true;
            wait_ans = true;
            countTimeout=0;
            CreateComAndSend(curCom);

            stateSendGsm =WAITANS;
            break;
        case WAITANS:
            countTimeout++;
            if (countTimeout>curCom.timeout){
                stateSendGsm =REPIT;
                countTimeout = 0;
            }
            if (!wait_ans){
                if (com_ok){
                    stateSendGsm =RESULTTRUE;
                }
                else {
                    stateSendGsm =RESULTFAIL;
                }
            }


            break;
        case REPIT:
            repicCount++;

            if (repicCount>=curCom.repitcount){
                stateSendGsm =RESULTFAIL;
                if (curCom.type==USOCO){
                    //modemParam.gprsConInit = true;
                }
            } else {
                stateSendGsm =SENDUART;

            }
            break;
        case RESULTFAIL:
            messSend = false;
            stateSendGsm = FIND;
            curCom.comState = curCom.Error;
            stateSendGsm =XZSEND;
            break;
        case RESULTTRUE:
            messSend = false;
            stateSendGsm = FIND;
            curCom.comState = OK;
            stateSendGsm =XZSEND;
            break;
        case XZSEND:
            break;
        default:
            break;
    }



}

ComGsm findCom(TypeCom typeCom){
    ComGsm find;
    int i =0;
    for ( i = 0; i<sizeof(comGsmTab)/sizeof(ComGsm); i++)
    {
        if (typeCom == comGsmTab[i].type){
            find = comGsmTab[i];
            break;
        }
    }
    return find;
}




void parseATstand(char *mess, size_t size){

            if(wait_ans_res){
                if(strstr(mess, okmess)){
                    wait_ans_res = false;
                    wait_ans = false;
                    com_ok = true;
                    //modemParam.parserMess = false;
                }
                if(strstr(mess, ermess)){
                    wait_ans = false;
                    wait_ans_res = false;
                    com_ok = false;
                   // modemParam.parserMess = false;
                }
            }


}

void parseATstandCpms(char *mess, size_t size){
            char* token;
            if(wait_ans_res){
                if(token=strstr(mess, "+CPMS:"))
                    {

                        token = (char*)mess+5;
                        token = strchr(token+1, ',');
                        modemParam.inputsms = (int)atoi(token+1);

                        if (modemParam.inputsms ==0){
                            modemParam.maxSMS = 0;
                            modemParam.errorSend = false;
                        }else{
                            if (modemParam.errorSend){
                                modemParam.inputsms = modemParam.maxSMS+MAX_SMS_DELTA;
                                modemParam.maxSMS+=MAX_SMS_DELTA;
                            }
                            if (modemParam.maxSMS<modemParam.inputsms){
                                modemParam.maxSMS = modemParam.inputsms;
                            }
                        }
                        System_printf("CREG inputsms = %d\r\n",modemParam.inputsms);
                        wait_ans_res = false;
                        wait_ans = false;
                        com_ok = true;

                    }
                if(strstr(mess, okmess)){
                    wait_ans_res = false;
                    wait_ans = false;
                    com_ok = true;


                }
                if(strstr(mess, ermess)){
                    wait_ans = false;
                    wait_ans_res = false;
                    com_ok = false;

                }
            }


}

void parseATstandCreg(char *mess, size_t size){



            if(wait_ans_res){
                char* token;
                if(token = strstr(mess, "+CREG: 2"))
                    {
                        //char* token;
                        char* tokenNew;
                        //System_printf("CREG\r\n");
                        //token = (char*)mess+8;
                        if (tokenNew = strchr(token+1, ','))
                        {
                            modemParam.registr = (int)atoi(tokenNew+1);
                            System_printf("CREG reg = %d\r\n",modemParam.registr);
                            token = tokenNew+1;
                            if (token = strchr(token, '"'))
                            {
                                tokenNew = strchr(token+1, '"');
                                strncpy(modemParam.lac, token+1, 4);
                                //System_printf("CREG lac = %s\r\n",modemParam.lac);
                                token = tokenNew+1;
                                token = strchr(token, '"');
                                tokenNew = strchr(token+1, '"');
                                strncpy(modemParam.sid, token+1, 4);
                            }
                            if (modemParam.registr==0 || modemParam.registr==2){
                                modemParam.gsmOff = true;
                                GsmPausa();
                            }
                            if (modemParam.registr==1){
                                modemParam.gsmOff = false;
                                modemParam.gsmRoaming = false;
                            }
                            if (modemParam.registr==5){
                                if (!modemParam.gsmRoaming){
                                    modemParam.gsmRoaming = true;
                                    GprsReConnect();
                                }
                            }
                        }

                    }
                if(strstr(mess, okmess)){
                    wait_ans_res = false;
                    wait_ans = false;
                    com_ok = true;

                }
                if(strstr(mess, ermess)){
                    wait_ans = false;
                    wait_ans_res = false;
                    com_ok = false;

                }
            }


}

void parseATstandCmgs(char *mess, size_t size){
    if(wait_ans_res){
        if (strstr(mess, ">")) {

            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;
            //int code = GetElementQueue();

            modemParam.messSMS = SetCode( modemParam.curSignal.code);
            modemParam.sizeSMS = GetSize( modemParam.curSignal.code);
            UartSend(modemParam.messSMS, modemParam.sizeSMS);
        }
    }
}




void parseATCMGR(char *mess, size_t size){
    if(wait_ans_res){
        char* tokParam;
        if(tokParam = strstr(mess, paramCode)){
           if (strstr(tokParam, "\n")){
               parsConfig(tokParam, size);
               //CopySMSmess(mess, size);
               //modemParam.busyRead = false;
           }
        }
        char* tokExe;
        if(tokExe = strstr(mess, execute)){
           if (strstr(tokExe, "\n")){
               parsExe(tokExe, size);
               //modemParam.busyRead = false;

           }
        }
        char* tokGpsg;
        if(tokGpsg = strstr(mess, gpsg)){
           if (strstr(tokGpsg, "\n")){
               parsGpsg(tokGpsg, size);
               //modemParam.busyRead = false;

           }
        }
        if(strstr(mess, okmess)){
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;
            modemParam.busyRead = false;
        }

        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;
            modemParam.busyRead = false;
        }



    }



}

void parseATCTRLZ(char *mess, size_t size){
    if(wait_ans_res){
            if(strstr(mess, okmess)){
                wait_ans_res = false;
                wait_ans = false;
                com_ok = true;
                modemParam.curSignal.flagsend = true;
            }
            if(strstr(mess, ermess)){
                wait_ans = false;
                wait_ans_res = false;
                com_ok = false;
                modemParam.curSignal.flagsend = false;
            }
        }


}

void parseATCOPS(char *mess, size_t size){
    if(wait_ans_res){
        if(strstr(mess, "+COPS:"))
        {
            if (strstr(mess, "\n")){
                //char* token;
                char* tokenNew;
                char* tokenEnd;

                if (tokenNew = strchr(mess, '"')){
                    strncpy(modemParam.mmc, tokenNew+1, 3);

                    tokenEnd = strchr(tokenNew+1, '"');
                    strncpy(modemParam.mnc, tokenNew+4, tokenEnd-tokenNew-4);
                }


            }

        }
        if(strstr(mess, okmess)){
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;

        }
        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;

        }
    }
}

void parseATstandGPRSInit(char *mess, size_t size){
    if(wait_ans_res){
        if(strstr(mess, okmess)){
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;
            modemParam.gprsInit = true;

        }
        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;
            modemParam.gprsInit = true;
        }
    }
   // modemParam.gprsInit = true;
}

void parseATstandInit(char *mess, size_t size){
    if(wait_ans_res){
        if(strstr(mess, okmess)){
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;
            modemParam.busyInit = false;
        }
        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;
            modemParam.busyInit = false;
        }
    }
   // modemParam.busyInit = false;
}

void parseATstandGprsCon(char *mess, size_t size){
    if(wait_ans_res){
        if(strstr(mess, okmess)){
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;
            modemParam.gprsConInit = true;
        }
        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;
            //modemParam.gprsConInit = true;
            //modemParam.gprsConFalse =true;
        }
    }
    //modemParam.gprsConInit = true;
}

void parseATstandUsowr(char *mess, size_t size){
    if(wait_ans_res){
        if (strstr(mess, "@")) {


            modemParam.messSMS = SetCodeGprs( modemParam.curSignal.code);
            modemParam.sizeSMS = GetSizeGprs();
            UartSend(modemParam.messSMS, modemParam.sizeSMS);
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;
            //modemParam.curSignal.flagsend = true;
        }
        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;

        }

    }
}

void parseATstandUsocr(char *mess, size_t size){
    if(wait_ans_res){
        if(strstr(mess, "USOCR:")){
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;

        }
        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;

        }
    }
}

void parseATUsowrOk(char *mess, size_t size){
    if(wait_ans_res){
        if(strstr(mess, okmess)){
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;
            modemParam.gprsParser = true;
            //modemParam.curSignal.flagsend = true;
            AddElementQueueAt(USOWROK);
            //modemParam.parserMess = false;
        }
        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;
            modemParam.gprsParser = true;

           // modemParam.parserMess = false;
        }
    }
}

void parseATstandUsord(char *mess, size_t size){
    if(wait_ans_res){
        //char token0;
        if (strstr(mess, "\"\"")){
            parseATstand(mess, size);
        }else{
            char* tokExe;
            if(tokExe = strstr(mess, execute)){
               char* tokenZero;
               if (tokenZero = strchr(tokExe, '\0')){
                   //if (modemParam.gprsParser){
                   parsExe(tokExe, size);
                       //modemParam.gprsParser = false;
                   //}
                   //modemParam.busyRead = false;
                   parseATUsowrOk(tokenZero+1, size);
               }
            }
            char* tokGpsg;
            if(tokGpsg = strstr(mess, gpsg)){
               char* tokenZero;
               if (tokenZero = strchr(tokGpsg, '\0')){
                   if (modemParam.gprsParser){
                       parsGpsg(tokGpsg, size);
                       modemParam.gprsParser = false;
                   }
                   //modemParam.busyRead = false;
                   parseATUsowrOk(tokenZero+1, size);
               }
            }
            char* tokAsk;

            if(tokAsk = strstr(mess, ask)){
               if (tokAsk = strchr(tokAsk, '\0')){
                   //parsGpsg(tokAsk, size);
                   //modemParam.busyRead = false;
                   modemParam.curSignal.flagsend = true;
                   parseATstand(tokAsk+1, size);
               }
            }
        }



    }
}

void parseATDiscon(char *mess, size_t size){
    if(wait_ans_res){
        if(strstr(mess, okmess)){
            wait_ans_res = false;
            wait_ans = false;
            com_ok = true;
            modemParam.gprsDisconFlag = true;
        }
        if(strstr(mess, ermess)){
            wait_ans = false;
            wait_ans_res = false;
            com_ok = false;

            modemParam.gprsDisconFlag = true;
        }
    }

}

void parseAT(char *mess, size_t size){
    if (messSend){
        if (curCom.type == AT){
            parseATstand(mess, size);

        }

        if (curCom.type == IPR){
            parseATstand(mess, size);

        }

        if (curCom.type == CPIN){
            parseATstand(mess, size);

        }
        if (curCom.type == COPS){
            parseATstand(mess, size);

        }
        if (curCom.type == COPS_A){
            parseATCOPS(mess, size);

        }
        if (curCom.type == CMGF){
            parseATstand(mess, size);

        }
        if (curCom.type == CSCS){
            parseATstand(mess, size);

        }
        if (curCom.type == CNMI){
            parseATstand(mess, size);

        }
        if (curCom.type == CPMS){
            parseATstand(mess, size);

        }
        if (curCom.type == CSQ){
            parseATstandInit(mess, size);

        }
        if (curCom.type == ATE0){
            parseATstand(mess, size);

        }
        if (curCom.type == CREG){
            parseATstand(mess, size);

        }
        if (curCom.type == CREG_A){
            parseATstandCreg(mess, size);

        }
        if (curCom.type == CPMS_A){
            parseATstandCpms(mess, size);

        }
        if (curCom.type == CMGS){
            parseATstandCmgs(mess, size);

        }
        if (curCom.type == CTRL_Z){
            parseATCTRLZ(mess, size);
            //parseATstand(mess, size);

        }
        if (curCom.type == UPSD0){
            parseATstand(mess, size);

        }
        if (curCom.type == UPSD1){
            parseATstand(mess, size);

        }
        if (curCom.type == UPSD2){
            parseATstand(mess, size);

        }
        if (curCom.type == UPSD3){
            parseATstandGPRSInit(mess, size);

        }
        if (curCom.type == CMGR){
            parseATCMGR(mess, size);
        }
        if (curCom.type == CMGD){
            parseATstand(mess, size);
        }
        if (curCom.type == UPSDA3){
            parseATstand(mess, size);

        }
        if (curCom.type == UPSDA4){
            parseATDiscon(mess, size);

        }
        if (curCom.type == USORD){
            parseATstandUsord(mess, size);

        }
        if (curCom.type == USOCR){
            parseATstandUsocr(mess, size);

        }
        if (curCom.type == USOWR){
            parseATstandUsowr(mess, size);

        }
        if (curCom.type == USOWROK){
            parseATstand(mess, size);

        }
        if (curCom.type == USOCO){
            parseATstandGprsCon(mess, size);

        }
        if (curCom.type == USOCL){
            parseATstand(mess, size);

        }

    }
}

