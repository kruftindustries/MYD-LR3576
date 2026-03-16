/*
 * main.c - Vehicle Monitor application entry point
 *
 * GtkStack + GtkStackSidebar navigation, page plugin registration,
 * database init, camera/sensor lifecycle management.
 */
#include "app.h"
#include "core/database.h"
#include "core/config.h"
#include "core/camera.h"
#include "core/clip.h"
#include "ui/util.h"
#include <libavutil/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

static volatile int sigint_flag = 0;

static void sigint_handler(int sig)
{
    (void)sig;
    sigint_flag = 1;
}

/* Suppress noisy FFmpeg HEVC decoder messages (PPS/NAL parse errors on
 * stream join before first keyframe are normal and expected) */
static void ffmpeg_log_callback(void *ptr, int level, const char *fmt, va_list vl)
{
    if (level > AV_LOG_WARNING)
        return;
    /* Filter out expected HEVC stream-join errors and swscaler noise */
    if (fmt && (strstr(fmt, "PPS id out of range") ||
                strstr(fmt, "Error parsing NAL") ||
                strstr(fmt, "deprecated pixel format")))
        return;
    av_log_default_callback(ptr, level, fmt, vl);
}

/* All pages in sidebar order */
static const PageDef *all_pages[] = {
    &page_cameras,
    &page_birdview,
    &page_sensors,
    &page_dvr,
    &page_config,
};
#define NUM_PAGES (int)(sizeof(all_pages) / sizeof(all_pages[0]))

static void on_visible_child_changed(GObject *stack, GParamSpec *pspec,
                                     gpointer data)
{
    (void)pspec;
    AppCtx *app = (AppCtx *)data;
    const char *name = gtk_stack_get_visible_child_name(GTK_STACK(stack));
    if (!name)
        return;

    for (int i = 0; i < NUM_PAGES; i++) {
        if (strcmp(all_pages[i]->id, name) == 0) {
            if (all_pages[i]->on_show)
                all_pages[i]->on_show(app);
        } else {
            if (all_pages[i]->on_hide)
                all_pages[i]->on_hide(app);
        }
    }
}

static gboolean check_sigint(gpointer data)
{
    AppCtx *app = (AppCtx *)data;
    if (sigint_flag) {
        app->running = 0;
        gtk_main_quit();
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

static void on_window_destroy(GtkWidget *widget, gpointer data)
{
    (void)widget;
    AppCtx *app = (AppCtx *)data;
    app->running = 0;
    gtk_main_quit();
}

static void cleanup_all(AppCtx *app)
{
    /* Stop clip transcoder before cameras (drains queue) */
    if (app->clip_transcoder) {
        clip_transcoder_stop(app->clip_transcoder);
        free(app->clip_transcoder);
        app->clip_transcoder = NULL;
    }

    /* Stop recordings first (sets recording=0, segment closes in worker) */
    for (int i = 0; i < app->num_cameras; i++) {
        if (app->cameras[i].recording)
            camera_rec_stop(&app->cameras[i], app);
    }

    /* Stop cameras */
    for (int i = 0; i < app->num_cameras; i++)
        camera_stop(&app->cameras[i]);

    /* Stop sensors */
    for (int i = 0; i < app->num_sensors; i++) {
        app->sensors[i].running = 0;
        if (app->sensors[i].id > 0)
            pthread_join(app->sensors[i].thread, NULL);
    }

    /* Page cleanup */
    for (int i = 0; i < NUM_PAGES; i++) {
        if (all_pages[i]->cleanup)
            all_pages[i]->cleanup(app);
    }

    /* Remove timers */
    if (app->stats_timer) {
        g_source_remove(app->stats_timer);
        app->stats_timer = 0;
    }

    /* Close database */
    if (app->db) {
        sqlite3_close(app->db);
        app->db = NULL;
    }

    /* Free camera resources */
    for (int i = 0; i < app->num_cameras; i++)
        camera_free(&app->cameras[i]);
}

int main(int argc, char *argv[])
{
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    av_log_set_callback(ffmpeg_log_callback);

    gtk_init(&argc, &argv);

    AppCtx app;
    memset(&app, 0, sizeof(app));
    app.running = 1;
    app.pages = all_pages;
    app.num_pages = NUM_PAGES;
    snprintf(app.db_path, sizeof(app.db_path), "vehicle_monitor.db");

    /* Initialize database */
    app.db = db_open(app.db_path);
    if (!app.db) {
        fprintf(stderr, "Failed to open database: %s\n", app.db_path);
        return 1;
    }

    /* Load configuration */
    config_load_all(&app);

    /* Initialize clip transcoder */
    app.clip_transcoder = calloc(1, sizeof(ClipTranscoder));
    if (app.clip_transcoder) {
        clip_transcoder_init(app.clip_transcoder);
        clip_transcoder_start(app.clip_transcoder);
    }

    /* Main window */
    app.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(app.window), "Vehicle Monitor");
    gtk_window_set_default_size(GTK_WINDOW(app.window), 1280, 720);
    g_signal_connect(app.window, "destroy",
                     G_CALLBACK(on_window_destroy), &app);

    /* Horizontal layout: sidebar | stack */
    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_container_add(GTK_CONTAINER(app.window), hbox);

    /* Stack for page content */
    app.stack = gtk_stack_new();
    gtk_stack_set_transition_type(GTK_STACK(app.stack),
                                 GTK_STACK_TRANSITION_TYPE_CROSSFADE);
    gtk_stack_set_transition_duration(GTK_STACK(app.stack), 150);

    /* Sidebar for navigation */
    app.sidebar = gtk_stack_sidebar_new();
    gtk_stack_sidebar_set_stack(GTK_STACK_SIDEBAR(app.sidebar),
                               GTK_STACK(app.stack));
    gtk_widget_set_size_request(app.sidebar, 160, -1);

    /* Style the sidebar */
    GtkCssProvider *css = gtk_css_provider_new();
    gtk_css_provider_load_from_data(css,
        "stacksidebar list { background: #2d2d2d; }"
        "stacksidebar list row { padding: 8px 12px; }"
        "stacksidebar list row label { color: #e0e0e0; font-size: 13px; }"
        "stacksidebar list row:selected { background: #4a90d9; }"
        "stacksidebar list row:selected label { color: white; }"
        "notebook header { background: #2d2d2d; }"
        "notebook header tab { padding: 6px 16px; color: #c0c0c0; }"
        "notebook header tab:checked { background: #3d3d3d; color: white; }"
        "notebook header tab:hover { background: #404040; }",
        -1, NULL);
    gtk_style_context_add_provider_for_screen(
        gdk_screen_get_default(),
        GTK_STYLE_PROVIDER(css),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(css);

    /* Add separator between sidebar and content */
    GtkWidget *sep = gtk_separator_new(GTK_ORIENTATION_VERTICAL);

    gtk_box_pack_start(GTK_BOX(hbox), app.sidebar, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), sep, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), app.stack, TRUE, TRUE, 0);

    /* Build and register all pages */
    for (int i = 0; i < NUM_PAGES; i++) {
        GtkWidget *page_widget = all_pages[i]->build(&app);
        gtk_stack_add_titled(GTK_STACK(app.stack), page_widget,
                             all_pages[i]->id, all_pages[i]->title);
    }

    /* Connect page-switch signal */
    g_signal_connect(app.stack, "notify::visible-child",
                     G_CALLBACK(on_visible_child_changed), &app);

    /* SIGINT check timer */
    g_timeout_add(200, check_sigint, &app);

    /* Show everything */
    gtk_widget_show_all(app.window);

    /* Trigger initial on_show for the first page */
    if (NUM_PAGES > 0 && all_pages[0]->on_show)
        all_pages[0]->on_show(&app);

    printf("Vehicle Monitor started (%d cameras, %d sensors)\n",
           app.num_cameras, app.num_sensors);
    fflush(stdout);

    gtk_main();

    printf("Shutting down...\n");
    cleanup_all(&app);
    printf("Done.\n");
    return 0;
}
