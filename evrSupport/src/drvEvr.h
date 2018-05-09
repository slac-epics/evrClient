#ifndef DRV_EVR_H
#define DRV_EVR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*EventTaskWorker)(void*);

int
eventTaskSendWork(EventTaskWorker worker, void *work);

int
eventTaskTrySendWork(EventTaskWorker worker, void *work);


#ifdef __cplusplus
};
#endif

#endif
