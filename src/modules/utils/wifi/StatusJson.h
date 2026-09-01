/*
 * StatusJson.h
 *
 * Builds the JSON document served by the read-only HTTP status API (GET /status).
 */

#ifndef STATUSJSON_H_
#define STATUSJSON_H_

#include <cstddef>

// Builds the /status JSON document into buf (always NUL-terminated).
// Returns the body length (excluding the NUL); output is truncated safely
// if buf_size is too small.
// Must only be called from main-loop context (ON_IDLE / ON_MAIN_LOOP),
// it performs PublicData reads.
size_t build_status_json(char *buf, size_t buf_size,
                         const char *machine_name,
                         const char *sta_ip,
                         int console_clients);

#endif /* STATUSJSON_H_ */
