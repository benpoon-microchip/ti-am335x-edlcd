#ifndef WORKQUEUE_H
#define WORKQUEUE_H

#include "list.h"
#include "mqueue.h"

#define INIT_WORK(_work, _func)				\
	do {								\
		INIT_LIST_HEAD(&(_work)->entry);			\
		(_work)->func = (_func);				\
	} while (0)


struct workqueue_struct {
	struct list_head list;
	mqd_t queue;
	const char *name;   /*workqueue name*/
};

struct work_struct{
	unsigned long pending;
	struct list_head entry;/* 將任務掛載到 queue 的掛載點 */
	//void (*func)(void *);  /* 任務方法 */
	void (*func)(struct work_struct *);
	void *data;            /* 任務處理的數據 */
	void *wq_data;         /* work 的屬主 */

};

int queue_work(struct workqueue_struct *wq, struct work_struct *work);
void flush_workqueue(struct workqueue_struct *wq);
void destroy_workqueue(struct workqueue_struct *wq);
struct workqueue_struct* create_singlethread_workqueue(char *queue_name);

#endif

