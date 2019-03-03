/*
 * gsm.h
 *
 *  Created on: 4 окт. 2018 г.
 *      Author: shestakov.aa
 */

#ifndef GSM_H_
#define GSM_H_

#include <stdbool.h>
#include <ti/drivers/UART.h>
#include "Signal.h"
#include "MyQueue.h"


typedef enum ComState {
    OK,
    ER,
    ER_INIT,
    ER_GPRS,
    ER_SENDSMS,
    ER_READSMS,
    ER_CON_GPRS,
    ER_WR_GPRS,
    ER_WROK_GPRS,
    ER_RD_GPRS,
    ER_DISCON_GPRS,
    NO
}ComState;


typedef enum StateSendGsm {
    FIND,
    SENDUART,
    WAITANS,
    RESULT,
    REPIT,
    RESULTFAIL,
    RESULTTRUE,
    XZSEND
}StateSendGsm;

typedef enum StatrSendSMS{
    CLEARSMS,
    SENDCMGS,
    WAITCMGS,
    SENDMESS,
    SENDCTRL,
    WAITSENDCTRL,
    FINISHSMSOK,
    FINISHSMSER
}StatrSendSMS;

typedef enum StateReadSMS{
    STARTREAD,
    SENDCMGR,
    WAITSENDCMGR,
    SENDCMGD,
    FINISHREADOK,
    FINISHREADER
}StateReadSMS;

typedef enum StateInitGSM {
    CLEAR,
    SENDSTACK,
    WAITINIT,
    WHATNEXT,
    FINISHOK,
    FINISHER,
    XZINIT
}StateInitGSM;

typedef enum StateGSM {
    INITGSM,
    WAITINITGSM,
    GPRSSEND,
    GPRSREAD,
    LOGIKASEND,
    RESETGSM,
    OPROSGSM,
    OPROSWAIT,
    WAITGPRS,
    MAINWAIT,
    SENDMESSGSM,
    READSMS,
    WAITREADSMS,
    WAITSENDMESSSMS,
    GSMTIMEOUT,
    XZSTATE
}StateGSM;

typedef enum TypeCom {
    AT,
    IPR,
    CPIN,
    CMGF,
    CSCS,
    COPS,
    COPS_A,
    CNMI,
    CPMS,
    CSQ,
    ATE0,
    CREG,
    CREG_A,
    CPMS_A,
    CMGS,
    CMGR,
    CMGD,
    CTRL_Z,
    UPSD0,
    UPSD1,
    UPSD2,
    UPSD3,
    UPSDA3,
    UPSDA4,
    UPSND0,
    USOCR,
    USOCL,
    USOWR,
    USOWROK,
    USOCO,
    USORD,
    XZ
}TypeCom;

typedef enum StateModemEr {
    OKMODEM,
    ERRORINIT,
    NOSTATE
}StateModemEr;

typedef enum StateGPRS {
    GPRSINIT,
    GPRSWAITINIT,
    GPRSSTART,
    GPRSMAIN,
    GPRSCONNECT,
    GPRSWAITCONNECT,
    GPRSDISCONECT,
    GPRSTIMEOUT


}StateGPRS;

typedef enum StateSend {

    SENDINIT,
    SENDWAIT,
    SENDGPRS,
    SENDSMS,
    SENDTIMEOUT,
    SENDXZ,
    SENDLOGICK,
    SENDEND,
    SENDWAITSMS,
    SENDWAITGPRS,
    SENDERRORGPRS

}StateSend;

typedef struct ComGsm {
    TypeCom type;
    char comand[42];
    int size;

    int timeout;
    int repitcount;
    ComState Error;
    ComState comState;

} ComGsm;

typedef struct ComStack {
    TypeCom type;
    ComState comState;
} ComStack;

typedef struct ModemParam {

    StateModemEr stateModemEr;
    bool falseInit;
    bool stateInit;
    bool busyInit;
    bool busySend;
    bool sendFlag;
    bool gprsOn;
    bool gprsSleep;
    int gprsTimeOut;
    int initTimeout;
    bool busyRead;
    bool errorSend;
    bool gprsInit;
    bool gprsFalseInit;
    bool gprsStateInit;
    bool gprsUUCLR;
    bool gprsConInit;
    bool gprsConFalse;
    bool gprsParser;
    bool gprsReconnect;
    bool gprsConni;
    bool gprsDiscon;
    bool gprsDisconFlag;
    int gprsErrorCoount;
    int timeoutAsk;
    bool sendGprsFlag;
    bool sendSmsFlag;

    bool gsmRoaming;
    bool gsmOff;
    //bool parserMess;
    int countOpros;
    int registr;
    int gsmTimeout;
    SignalStr curSignal;


    char lac[4];
    char sid[4];
    char mmc[4];
    char mnc[4];
    int inputsms;

    int maxSMS;
    //bool sendSMS;
    char* messSMS;
    int sizeSMS;
    StateGSM stateGSM;
    StateSend stageSend;
} ModemParam;



StateGPRS stateGprs;
ModemParam modemParam;
bool wait_ans_com;
bool wait_ans_res;
bool com_ok;
bool wait_ans;
bool messSend;
bool busySendAt;
ComGsm curCom;
void GprsDiscon(void);
void  ParserQuick(void);
void UpdateComand(void);
void SendModul(void);
ComState InitModemGPRS(void);
void parseATstandCmgs(char *mess, size_t size);
void GprsReConnect(void);
void parseATDiscon(char *mess, size_t size);
void UpdateGsmState(void);
void GsmInit(void);
void GsmPausa(void);
void GsmTaskInit(void);
void parseATstandCpms(char *mess, size_t size);
void parseATstandUsowr(char *mess, size_t size);
void parseATstandUsocr(char *mess, size_t size);
void GsmSetState(bool st);
void Gsm_start(void);
void Gsm_stop(void);
void  GprsModul(void);
void parseATstandInit(char *mess, size_t size);

void parseATstand(char *mess, size_t size);
void parseAT(char *mess, size_t size);
void SendCommGSM(TypeCom );
ComGsm findCom(TypeCom );
void parseATstandCreg(char *mess, size_t size);
void toArray(int number, char *numberArray, int size);
void PastComU(TypeCom typeCom, char *past);
void ParserError(ComState comstateEr);
void CreateComAndSend(ComGsm com);
void parseATCMGR(char *mess, size_t size);
void parseATCTRLZ(char *mess, size_t size);
void parseATUsowrOk(char *mess, size_t size);
void CreateParamCode(void);

#endif /* GSM_H_ */
