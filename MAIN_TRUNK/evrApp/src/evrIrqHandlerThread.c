#include <signal.h>
#include <unistd.h>
#include <epicsThread.h>
#include <errlog.h>

static int evrIrqHandlerThread(void *handler)
{
    void (*irqHandler)(int) = handler;
    int signal;
    sigset_t  sigSet;

    sigemptyset(&sigSet);
    sigaddset(&sigSet,SIGIO);

    while(1) {
        sigwait(&sigSet, &signal); 
        if(irqHandler) irqHandler(signal);
    }

    return 0;
}

void EvrIrqHandlerThreadCreate(void (*handler) (int))
{
    epicsThreadMustCreate("evrIrqHandler", epicsThreadPriorityHigh+5,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)evrIrqHandlerThread,handler);
}

static struct EvrIrqFdStruct {
    int fd;
    void *handler;
} evrIrqFdStruct;

static int evrIrqFdHandlerThread(void *eiFdStruct) {
    struct EvrIrqFdStruct *eiFdS = eiFdStruct;
    int val, retR;
    void (*irqHandler)(int) = eiFdS->handler;

    while (1) {
        retR = read(eiFdS->fd, &val, 4);
        if (retR != 4) {
            errlogPrintf("Read failed, return value: %d\n", retR);
            return -1;
        }
        if (irqHandler) {
            irqHandler(val);
        }
    }
    return 0;
}

void EvrIrqFdHandlerThreadCreate(int fd, void (*handler)(int)) {
    evrIrqFdStruct.fd = fd;
    evrIrqFdStruct.handler = handler;

    epicsThreadMustCreate("evrIrqFdHandler", epicsThreadPriorityHigh + 5,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC) evrIrqFdHandlerThread, &evrIrqFdStruct);
}
