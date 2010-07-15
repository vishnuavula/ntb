#undef TRACE_SYSTEM
#define TRACE_SYSTEM raid5

#if !defined(_TRACE_RAID5_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_RAID5_H

#include <md.h>
#include <raid5.h>
#include <linux/tracepoint.h>

TRACE_EVENT(raid5_handle_list,

	TP_PROTO(raid5_conf_t *conf, struct stripe_head *sh),

	TP_ARGS(conf, sh),

	TP_STRUCT__entry(
		__array(char,		name,	DISK_NAME_LEN)
		__field(sector_t,	sector)
	),

	TP_fast_assign(
		memcpy(__entry->name, mdname(conf->mddev), DISK_NAME_LEN);
		__entry->sector = sh->sector;
	),

	TP_printk("mddev=%s sector=%llu", __entry->name,
		  (unsigned long long) __entry->sector)
);

TRACE_EVENT(raid5_hold_list,

	TP_PROTO(raid5_conf_t *conf, struct stripe_head *sh),

	TP_ARGS(conf, sh),

	TP_STRUCT__entry(
		__array(char,		name,	DISK_NAME_LEN)
		__field(sector_t,	sector)
		__field(int,		full)
	),

	TP_fast_assign(
		memcpy(__entry->name, mdname(conf->mddev), DISK_NAME_LEN);
		__entry->full = atomic_read(&conf->pending_full_writes);
		__entry->sector = sh->sector;
	),

	TP_printk("mddev=%s sector=%llu full: %d",
		  __entry->name, (unsigned long long) __entry->sector,
		  __entry->full)
);

TRACE_EVENT(raid5_hold_delay,

	TP_PROTO(raid5_conf_t *conf, struct stripe_head *sh, unsigned long tmo),

	TP_ARGS(conf, sh, tmo),

	TP_STRUCT__entry(
		__array(char,		name,	DISK_NAME_LEN)
		__field(sector_t,	sector)
		__field(unsigned long,	tmo)
	),

	TP_fast_assign(
		memcpy(__entry->name, mdname(conf->mddev), DISK_NAME_LEN);
		__entry->sector = sh->sector;
		__entry->tmo = tmo;
	),

	TP_printk("mddev=%s sector=%llu tmo: %lu", __entry->name,
		  (unsigned long long) __entry->sector, __entry->tmo)
);

TRACE_EVENT(raid5_hold_enter,

	TP_PROTO(raid5_conf_t *conf, struct stripe_head *sh),

	TP_ARGS(conf, sh),

	TP_STRUCT__entry(
		__array(char,		name,	DISK_NAME_LEN)
		__field(sector_t,	sector)
	),

	TP_fast_assign(
		memcpy(__entry->name, mdname(conf->mddev), DISK_NAME_LEN);
		__entry->sector = sh->sector;
	),

	TP_printk("mddev=%s sector=%llu", __entry->name,
		  (unsigned long long) __entry->sector)
);

TRACE_EVENT(raid5_delay_enter,

	TP_PROTO(raid5_conf_t *conf, struct stripe_head *sh),

	TP_ARGS(conf, sh),

	TP_STRUCT__entry(
		__array(char,		name,	DISK_NAME_LEN)
		__field(sector_t,	sector)
	),

	TP_fast_assign(
		memcpy(__entry->name, mdname(conf->mddev), DISK_NAME_LEN);
		__entry->sector = sh->sector;
	),

	TP_printk("mddev=%s sector=%llu", __entry->name,
		  (unsigned long long) __entry->sector)
);

#endif /* _TRACE_RAID5_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE raid5-trace
#include <trace/define_trace.h>
