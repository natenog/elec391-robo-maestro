#ifndef __FSM_H
#define __FSM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum {
	HOME,
	MOVE,
	PLAY,
	DONE
} FSMState;

#ifdef __cplusplus
}
#endif

#endif
