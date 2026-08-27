/*
 * fsevents.c — OS-native filesystem-event wake-up trigger.
 *
 * Linux: inotify, recursively watching every subdirectory under a project's
 * root_path (skipping the same directories the indexer already ignores —
 * .git, node_modules, build output, etc. — via cbm_should_skip_dir).
 * Directory creation adds a watch dynamically (inotify does not recurse).
 * Runs its own thread blocked on read(); a self-pipe lets cbm_fsevents_stop
 * wake it for shutdown. Events are coalesced by a quiet-period debounce so a
 * burst (e.g. `git checkout` touching hundreds of files) produces at most one
 * cbm_watcher_touch() call per quiet window.
 *
 * macOS/BSD (kqueue) and Windows (ReadDirectoryChangesW) are not implemented
 * yet: cbm_fsevents_start returns NULL there, which is the documented,
 * expected "backend unavailable" fallback — callers keep interval-only
 * polling, unchanged from today's behavior.
 *
 * This file MUST NOT decide "this is a real change" or call index_fn. It
 * only resets next_poll_ns via cbm_watcher_touch(); the existing git
 * status/signature poll path in watcher.c remains the sole source of truth.
 */
#include "watcher/fsevents.h"
#include "watcher/watcher.h"
#include "discover/discover.h"
#include "foundation/compat_fs.h"
#include "foundation/compat_thread.h"
#include "foundation/log.h"

#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

#if defined(__linux__)

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/inotify.h>
#include <unistd.h>

/* Quiet-period debounce: a touch fires only after this many ms with no
 * further inotify activity, so a large burst collapses to one touch. */
#define FSEVENTS_DEBOUNCE_MS 300
#define FSEVENTS_WATCH_MASK \
    (IN_CREATE | IN_DELETE | IN_MODIFY | IN_MOVED_FROM | IN_MOVED_TO | IN_ATTRIB | IN_CLOSE_WRITE)

typedef struct {
    int wd;
    char *path; /* absolute path this watch descriptor covers */
} wd_entry_t;

struct cbm_fsevents {
    cbm_watcher_t *w;
    char *project_name;
    char *root_path;
    int inotify_fd;
    int stop_pipe[2]; /* self-pipe: [0]=read, [1]=write, wakes the thread for shutdown */
    cbm_thread_t thread;
    wd_entry_t *watches;
    int watch_count;
    int watch_cap;
    atomic_bool running;
};

static bool wd_table_add(cbm_fsevents_t *fe, int wd, const char *path) {
    if (fe->watch_count >= fe->watch_cap) {
        int new_cap = fe->watch_cap ? fe->watch_cap * 2 : 32;
        wd_entry_t *tmp = realloc(fe->watches, (size_t)new_cap * sizeof(*tmp));
        if (!tmp) {
            return false;
        }
        fe->watches = tmp;
        fe->watch_cap = new_cap;
    }
    char *copy = strdup(path);
    if (!copy) {
        return false;
    }
    fe->watches[fe->watch_count].wd = wd;
    fe->watches[fe->watch_count].path = copy;
    fe->watch_count++;
    return true;
}

static const char *wd_table_lookup(cbm_fsevents_t *fe, int wd) {
    for (int i = 0; i < fe->watch_count; i++) {
        if (fe->watches[i].wd == wd) {
            return fe->watches[i].path;
        }
    }
    return NULL;
}

static void wd_table_free(cbm_fsevents_t *fe) {
    for (int i = 0; i < fe->watch_count; i++) {
        free(fe->watches[i].path);
    }
    free(fe->watches);
    fe->watches = NULL;
    fe->watch_count = 0;
    fe->watch_cap = 0;
}

/* Add a watch on dir_path and, on success, recurse into its children
 * (skipping ignored directories, symlinks, and anything not a directory).
 * Returns false only on resource exhaustion (ENOSPC/EMFILE) — the caller
 * treats that as fatal setup failure so the project falls back cleanly to
 * polling; any other per-directory failure (permission, race with deletion)
 * is skipped and does not abort the whole watch. */
static bool add_watch_recursive(cbm_fsevents_t *fe, const char *dir_path) {
    int wd = inotify_add_watch(fe->inotify_fd, dir_path, FSEVENTS_WATCH_MASK);
    if (wd < 0) {
        if (errno == ENOSPC || errno == EMFILE || errno == ENOMEM) {
            return false;
        }
        return true; /* e.g. EACCES, ENOENT (raced away) — just skip this dir */
    }
    if (!wd_table_add(fe, wd, dir_path)) {
        (void)inotify_rm_watch(fe->inotify_fd, wd);
        return false;
    }

    cbm_dir_t *d = cbm_opendir(dir_path);
    if (!d) {
        return true;
    }
    cbm_dirent_t *ent;
    while ((ent = cbm_readdir(d)) != NULL) {
        if (!ent->is_dir) {
            continue;
        }
        if (cbm_should_skip_dir(ent->name, CBM_MODE_FULL)) {
            continue;
        }
        char child[4096];
        int written = snprintf(child, sizeof(child), "%s/%s", dir_path, ent->name);
        if (written <= 0 || (size_t)written >= sizeof(child)) {
            continue;
        }
        if (!add_watch_recursive(fe, child)) {
            cbm_closedir(d);
            return false;
        }
    }
    cbm_closedir(d);
    return true;
}

/* Drain every event currently queued on the inotify fd (non-blocking).
 * Newly-created directories get a fresh recursive watch so subsequent
 * activity inside them is seen (inotify does not watch new subdirs
 * automatically). Returns true if at least one event was read. */
static bool drain_events(cbm_fsevents_t *fe) {
    char buf[4096] __attribute__((aligned(__alignof__(struct inotify_event))));
    bool any = false;
    for (;;) {
        ssize_t n = read(fe->inotify_fd, buf, sizeof(buf));
        if (n <= 0) {
            break;
        }
        any = true;
        ssize_t off = 0;
        while (off < n) {
            const struct inotify_event *ev = (const struct inotify_event *)(buf + off);
            if ((ev->mask & IN_CREATE) && (ev->mask & IN_ISDIR) && ev->len > 0) {
                const char *parent = wd_table_lookup(fe, ev->wd);
                if (parent && !cbm_should_skip_dir(ev->name, CBM_MODE_FULL)) {
                    char child[4096];
                    int written = snprintf(child, sizeof(child), "%s/%s", parent, ev->name);
                    if (written > 0 && (size_t)written < sizeof(child)) {
                        /* Best-effort: a failure here just means events under
                         * this new subdirectory are missed until the next
                         * poll's git status catches up (correctness intact). */
                        (void)add_watch_recursive(fe, child);
                    }
                }
            }
            off += (ssize_t)sizeof(struct inotify_event) + ev->len;
        }
    }
    return any;
}

static void *fsevents_thread_main(void *arg) {
    cbm_fsevents_t *fe = arg;
    struct pollfd fds[2];
    fds[0].fd = fe->inotify_fd;
    fds[0].events = POLLIN;
    fds[1].fd = fe->stop_pipe[0];
    fds[1].events = POLLIN;

    while (atomic_load_explicit(&fe->running, memory_order_acquire)) {
        fds[0].revents = 0;
        fds[1].revents = 0;
        int rc = poll(fds, 2, -1);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (fds[1].revents & POLLIN) {
            break; /* stop requested */
        }
        if (!(fds[0].revents & POLLIN)) {
            continue;
        }
        (void)drain_events(fe);

        /* Quiet-period debounce: keep draining until a poll cycle elapses
         * with no further inotify activity, then touch exactly once. */
        for (;;) {
            fds[0].revents = 0;
            fds[1].revents = 0;
            int wrc = poll(fds, 2, FSEVENTS_DEBOUNCE_MS);
            if (wrc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;
            }
            if (fds[1].revents & POLLIN) {
                return NULL; /* stop requested mid-burst */
            }
            if (wrc == 0) {
                /* Quiet period elapsed: wake the poller for its next cycle. */
                cbm_watcher_touch(fe->w, fe->project_name);
                break;
            }
            if (fds[0].revents & POLLIN) {
                (void)drain_events(fe);
                continue; /* activity kept coming — extend the quiet window */
            }
        }
    }
    return NULL;
}

cbm_fsevents_t *cbm_fsevents_start(cbm_watcher_t *w, const char *project_name,
                                   const char *root_path) {
    if (!w || !project_name || !root_path || !root_path[0]) {
        return NULL;
    }

    cbm_fsevents_t *fe = calloc(1, sizeof(*fe));
    if (!fe) {
        return NULL;
    }
    fe->w = w;
    fe->project_name = strdup(project_name);
    fe->root_path = strdup(root_path);
    fe->inotify_fd = -1;
    fe->stop_pipe[0] = -1;
    fe->stop_pipe[1] = -1;
    if (!fe->project_name || !fe->root_path) {
        cbm_fsevents_stop(fe);
        return NULL;
    }

    fe->inotify_fd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (fe->inotify_fd < 0) {
        cbm_log_warn("watcher.fsevents.init_failed", "project", project_name, "reason", "inotify_init1");
        cbm_fsevents_stop(fe);
        return NULL;
    }
    if (pipe(fe->stop_pipe) != 0) {
        cbm_log_warn("watcher.fsevents.init_failed", "project", project_name, "reason", "pipe");
        cbm_fsevents_stop(fe);
        return NULL;
    }
    (void)fcntl(fe->stop_pipe[0], F_SETFD, FD_CLOEXEC);
    (void)fcntl(fe->stop_pipe[1], F_SETFD, FD_CLOEXEC);

    if (!add_watch_recursive(fe, fe->root_path)) {
        cbm_log_warn("watcher.fsevents.init_failed", "project", project_name, "reason",
                     "watch_exhausted");
        cbm_fsevents_stop(fe);
        return NULL;
    }
    if (fe->watch_count == 0) {
        /* root_path itself could not be watched at all. */
        cbm_fsevents_stop(fe);
        return NULL;
    }

    atomic_init(&fe->running, true);
    if (cbm_thread_create(&fe->thread, 0, fsevents_thread_main, fe) != 0) {
        cbm_log_warn("watcher.fsevents.init_failed", "project", project_name, "reason",
                     "thread_create");
        atomic_store(&fe->running, false);
        cbm_fsevents_stop(fe);
        return NULL;
    }

    cbm_log_info("watcher.fsevents.started", "project", project_name, "watches",
                 fe->watch_count > 0 ? "yes" : "0");
    return fe;
}

void cbm_fsevents_stop(cbm_fsevents_t *fe) {
    if (!fe) {
        return;
    }
    bool was_running = atomic_exchange(&fe->running, false);
    if (was_running) {
        /* Wake the blocked poll() so the thread observes running=false. */
        if (fe->stop_pipe[1] >= 0) {
            char byte = 0;
            (void)!write(fe->stop_pipe[1], &byte, 1);
        }
        (void)cbm_thread_join(&fe->thread);
    }
    if (fe->inotify_fd >= 0) {
        close(fe->inotify_fd);
    }
    if (fe->stop_pipe[0] >= 0) {
        close(fe->stop_pipe[0]);
    }
    if (fe->stop_pipe[1] >= 0) {
        close(fe->stop_pipe[1]);
    }
    wd_table_free(fe);
    free(fe->project_name);
    free(fe->root_path);
    free(fe);
}

#else /* !__linux__: kqueue (macOS/BSD) and ReadDirectoryChangesW (Windows)
       * are not implemented yet. Returning NULL is the documented backend-
       * unavailable path — cbm_watcher_watch() falls back to interval-only
       * polling for the project, exactly as it did before this feature. */

struct cbm_fsevents {
    int unused;
};

cbm_fsevents_t *cbm_fsevents_start(cbm_watcher_t *w, const char *project_name,
                                   const char *root_path) {
    (void)w;
    (void)root_path;
    if (project_name) {
        cbm_log_info("watcher.fsevents.unavailable", "project", project_name, "reason",
                     "unsupported_platform");
    }
    return NULL;
}

void cbm_fsevents_stop(cbm_fsevents_t *fe) {
    free(fe);
}

#endif
