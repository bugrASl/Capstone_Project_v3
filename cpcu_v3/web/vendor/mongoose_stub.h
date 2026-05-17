/*
 *  mongoose_stub.h — minimal Mongoose API surface used by cpcu_ws,
 *                    sufficient to satisfy the compiler when the real
 *                    Mongoose hasn't been fetched yet.
 *
 *  REAL mongoose.h (10K+ lines) goes here after running fetch.sh.
 *  This file exists so syntax/structure of cpcu_ws.c can be validated
 *  in CI / offline development environments where curl can't reach
 *  GitHub. CMakeLists.txt will pick the real mongoose.{c,h} when
 *  present and ignore this stub.
 *
 *  DO NOT commit a build that links against this stub — it's a
 *  parser-pleaser, not a working WebSocket library.
 */

#ifndef MONGOOSE_STUB_H
#define MONGOOSE_STUB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Event types we use. Real values from upstream don't matter to the
 * stub — only that they're distinct ints. */
#define MG_EV_OPEN          1
#define MG_EV_HTTP_MSG      2
#define MG_EV_WS_OPEN       3
#define MG_EV_WS_MSG        4
#define MG_EV_CLOSE         5
#define MG_EV_POLL          6

#define WEBSOCKET_OP_TEXT   1
#define WEBSOCKET_OP_BINARY 2

/* Mongoose forward-decl pattern; we only touch fields cpcu_ws.c needs. */
struct mg_fs;
struct mg_str  { const char *buf; size_t len; };
struct mg_mgr  { struct mg_connection *conns; };

struct mg_connection {
    struct mg_connection *next;
    bool                  is_websocket;
    bool                  is_listening;
    bool                  is_closing;
    void                 *fn_data;     /* user pointer */
};

struct mg_http_message {
    struct mg_str method, uri, query, body;
};
struct mg_ws_message {
    struct mg_str data;
    uint8_t flags;
};

typedef void (*mg_event_handler_t)(struct mg_connection *c, int ev,
                                    void *ev_data);

void  mg_mgr_init(struct mg_mgr *mgr);
void  mg_mgr_free(struct mg_mgr *mgr);
void  mg_mgr_poll(struct mg_mgr *mgr, int timeout_ms);

struct mg_connection *mg_http_listen(struct mg_mgr *mgr, const char *url,
                                      mg_event_handler_t fn, void *fn_data);

struct mg_http_serve_opts {
    const char *root_dir;
    const char *ssi_pattern;
    const char *extra_headers;
    const char *mime_types;
    const char *page404;
    struct mg_fs *fs;
};

void  mg_http_serve_dir(struct mg_connection *c, struct mg_http_message *hm,
                        const struct mg_http_serve_opts *opts);

void  mg_ws_upgrade(struct mg_connection *c, struct mg_http_message *hm,
                    const char *fmt);
size_t mg_ws_send(struct mg_connection *c, const void *buf, size_t len,
                  int op);

bool mg_match(struct mg_str s, struct mg_str pat, struct mg_str *caps);
struct mg_str mg_str(const char *s);

void mg_log_set(int level);
#define MG_LL_INFO  2
#define MG_LL_DEBUG 3

#endif /* MONGOOSE_STUB_H */
