#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

void lorawan_set_conf(const char *appEui, const char *devEui, const char *appKey);
void lorawan_get_conf(char *appEui, char *devEui, char *appKey, uint8_t *status);

void davis_set_conf(uint16_t send_interval, bool stations[8], bool debug_trace);
void davis_get_conf(uint16_t *send_interval, bool stations[8], uint8_t *status, bool *debug_trace);

#endif /* MAIN_MAIN_H_ */
