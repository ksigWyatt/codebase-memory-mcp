/*
 * fsevents.h — OS-native filesystem-event wake-up trigger for the watcher.
 *
 * Purely a latency optimization: on filesystem activity under a watched
 * project's root, calls cbm_watcher_touch() to reset that project's adaptive
 * poll timer to zero so the NEXT scheduled poll runs immediately instead of
 * waiting out the interval. It never decides "this is a real change" and
 * never calls index_fn itself — all change confirmation still goes through
 * the existing git status/signature poll path in watcher.c.
 *
 * Backend: inotify on Linux. Other platforms fall back to interval-only
 * polling (cbm_fsevents_start returns NULL) — never an error, never a crash.
 */
#ifndef CBM_FSEVENTS_H
#define CBM_FSEVENTS_H

typedef struct cbm_watcher cbm_watcher_t;
typedef struct cbm_fsevents cbm_fsevents_t;

/* Start a background filesystem-event watcher for root_path, recursively.
 * On activity (excluding the same skip-list the indexer uses — .git,
 * node_modules, build output, etc.), calls cbm_watcher_touch(w, project_name)
 * after a short debounce so a burst of events (e.g. a git checkout touching
 * hundreds of files) triggers at most one touch per quiet period.
 *
 * w and project_name must outlive the returned handle; project_name is
 * copied. Returns NULL when the native backend is unavailable or setup
 * failed (resource exhaustion, unsupported platform) — callers MUST treat
 * that as a normal fallback to interval-only polling, not an error. */
cbm_fsevents_t *cbm_fsevents_start(cbm_watcher_t *w, const char *project_name,
                                   const char *root_path);

/* Stop the background thread and free all resources. NULL-safe. */
void cbm_fsevents_stop(cbm_fsevents_t *fe);

#endif /* CBM_FSEVENTS_H */
