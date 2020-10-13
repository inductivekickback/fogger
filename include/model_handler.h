/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/**
 * @file
 * @brief Model handler
 */

#ifndef MODEL_HANDLER_H__
#define MODEL_HANDLER_H__

#include <bluetooth/mesh.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELEMENT_COUNT 2

/* Called whenever an element receives a 'set' command */
typedef void (*model_handler_set_cb)(uint8_t elem_indx, bool status);

/**
 * @brief Initialize GPIO and workqueue items
 *
 * @retval  0 success
 * @retval -1 set_cb is NULL
 */
const int model_handler_init(const struct bt_mesh_comp **p_comp, model_handler_set_cb p_set_cb);

/**
 * @brief Set the specified element's status
 *
 * @retval  0 success
 * @retval -1 module has not been initialized
 * @retval -2 element_index is invalid
 */
int model_handler_elem_update(uint8_t elem_indx, bool status);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_HANDLER_H__ */
