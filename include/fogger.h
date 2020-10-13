/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef FOGGER_H__
#define FOGGER_H__

enum fogger_state {
	FOGGER_STATE_HEATING,
	FOGGER_STATE_READY,
	FOGGER_STATE_FOGGING
};

/* Called whenever the fogger changes state */
typedef void (*fogger_status_cb)(enum fogger_state);

/**
 * @brief Initialize GPIO and internal state
 *
 * @retval  0 success
 * @retval -2 GPIO init failed
 */
int fogger_init(fogger_status_cb p_status_cb);

/**
 * @brief Get the current state of the fogger
 *
 * @retval -1 module has not been initialized
 * @retval -2 state is NULL
 */
int fogger_state_get(enum fogger_state *p_state);

/**
 * @brief Start producing fog, if possible
 *
 * Attempt to start the fog machine. Has no effect if the machine is currently
 * active or is not ready because it is heating.
 *
 * @retval -1 module has not been initialized
 */
int fogger_start(void);

/**
 * @brief Stop producing fog
 *
 * Ensure that the fog machine is not producing fog. Has no effect if the
 * machine is not currently fogging.
 *
 * @retval -1 module has not been initialized
 */
int fogger_stop(void);

#endif /* FOGGER_H__ */
