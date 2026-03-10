/**
 * @file demo_sle.h
 * @brief SLE Module Interface (Server/Client)
 */

#ifndef DEMO_SLE_H
#define DEMO_SLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int demo_sle_init(void);
bool demo_sle_is_connected(void);
uint32_t demo_sle_tx_process(void);
uint16_t demo_sle_get_mtu(void);
uint16_t demo_sle_get_conn_id(void);
uint16_t demo_sle_get_effective_payload(void);
uint16_t demo_sle_get_fragment_payload(void);
void demo_sle_process_deferred(void);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_SLE_H */
