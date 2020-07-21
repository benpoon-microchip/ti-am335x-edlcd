#include <mqueue.h>

#include <list.h>
#include <sys/slogcodes.h>
#include <sched.h>
#include <workqueue.h>
#include <pthread.h>



struct workqueue_struct *test_wq = NULL;

void workqueue_handler(void *data)
{
	struct workqueue_struct *workqueue = (struct workqueue_struct *) data;
	unsigned int prio;
	ssize_t bytes_read;
	struct work_struct *work;
	//struct host_if_msg *msg;
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  workqueue = %p list = %p\n", __func__, workqueue, &workqueue->list);

	while (1)
	{

		///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] workqueue = %p list = %p, next =%p\n", __func__, workqueue, &workqueue->list, workqueue->list.next);
		if (!list_empty(&workqueue->list))
		{
			//printf("Enter...\r\n");
			//int transmit_pwr_mode;
			//scanf("%d",&transmit_pwr_mode);
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] have data\n", __func__);
			work = list_first_entry(&workqueue->list, struct work_struct, entry);
			//msg = container_of(work, struct host_if_msg, work);
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] work =%p\n", __func__, work);
			work->func(work);
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] quit fn\n", __func__);
			list_del(&work->entry);
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] delete entry\n", __func__);
		}
		usleep(10);

	}
#if 0
	while (1)
	{

		bytes_read = mq_receive(workqueue->queue, (char*) &buffer, sizeof(struct work_struct), &prio);

		//if (list_empty(	&workqueue->list) == false)
		if (bytes_read >= 0)
		{
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  Received message\n", __func__);
		} else {



		}
	}
#endif

}

struct workqueue_struct* create_singlethread_workqueue(char *queue_name)
{
	struct mq_attr q_attr;
	pthread_attr_t		t_attr;
	struct sched_param	sc_param;
	pthread_t tid;
	struct workqueue_struct* wq;

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In\n", __func__);

#if 1
	wq = create_ptr(sizeof (struct workqueue_struct));

	q_attr.mq_flags = 0;
	q_attr.mq_maxmsg = 10;
	q_attr.mq_msgsize = 256;
	q_attr.mq_curmsgs = 0;

	if ((wq->queue = mq_open(queue_name, O_CREAT | O_RDWR, 0644, &q_attr)) ==  -1)
	{
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  mq_open fail %d\n", __func__, errno);
	}
#endif

	INIT_LIST_HEAD(&wq->list);

	//LIST_HEAD(list_head);
	//wq->list = &list_head;
	//wq->name = queue_name;


	pthread_attr_init(&t_attr);
	pthread_attr_setschedpolicy(&t_attr, SCHED_RR);


	//sc_param.sched_priority = 1;
	//pthread_attr_setschedparam(&t_attr, &sc_param);
	pthread_attr_setinheritsched(&t_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&t_attr, PTHREAD_CREATE_DETACHED);
	pthread_attr_setstacksize(&t_attr, 8192);


	/* Create SDIO event handler */
	if (pthread_create(&tid, &t_attr, (void *)workqueue_handler, wq)) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  Unable to create event pthread\n", __func__);
		return -1;
	}

	int ret = pthread_getschedparam(tid, NULL, &sc_param );
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  get priority = %d,  sched_curpriority = %d, ret = %d\n", __func__, sc_param.sched_priority, sc_param.sched_curpriority, ret);


	return wq;
}



void queue_work(struct workqueue_struct *wq, struct work_struct *work)
{
	unsigned int prio = 0;
	int ret = 0;

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  workqueue = %p, work = %p, wq->list = %p, work->entry = %p\n", __func__, wq, work, &wq->list, work->entry);

	list_add_tail(&work->entry,&wq->list);

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  wq->list.next = %p\n", __func__, wq->list.next);

	//ret = mq_send(wq->queue, (char*) work, sizeof(struct work_struct), prio);
	//ret = mq_send(wq->queue, "Hello Test", 10, prio);
	//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  ret = %d, error=%d\n", __func__, ret, errno);

}

void flush_workqueue(struct workqueue_struct *wq)
{
	// ToDO
}

void destroy_workqueue(struct workqueue_struct *wq)
{
	// ToDO
}


