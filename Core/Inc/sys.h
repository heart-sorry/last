#include "stdint.h"//不加这个无法使用uint3_t等关键词
#ifndef __SYS_H__
#define __SYS_H__

/* =============================状态机============================= */
/* 状态机标志位 */
typedef enum
{
    SYS_STA_INIT = 0,        // 初始化
    SYS_STA_DEBUG,           // 调试
    SYS_STA_RUN,             // 运行
    SYS_STA_STOP,            // 停止
    // --------
    SYS_STA_MAX_NUM          // 状态机最大状态数
} SYS_FLAG_STA_E;

extern SYS_FLAG_STA_E g_eSysFlagManage;		// 状态机标志位

/* 状态机对应的任务 */
typedef struct
{
	uint8_t	 SysStaID;			    // 当前任务所属的状态机
    uint32_t Run;					// 任务定时时间到了，可以开始执行
    uint32_t TimCount;				// 任务计时器初始值
    uint32_t TimRload;				// 任务计时器重载值
    void (*pTaskFunc)(void);		// 该任务对应的函数
} TaskComps_T;


/* =============================系统函数============================= */
extern void SysInit(void);
extern void SysRun(void);

extern void TaskTimeSliceManage(void);


#endif /* __SYS_H */
