/*
 * pages/page_config.c - Tabbed configuration wrapper
 *
 * GtkNotebook combining Camera Config, Motion Config,
 * BirdView Config, and Sensor Config as tabs.
 */
#include "../app.h"
#include "../ui/util.h"
#include <stdlib.h>
#include <string.h>

#define CONFIG_NUM_TABS 7

typedef struct {
    AppCtx          *app;
    GtkWidget       *notebook;
    const PageDef   *tab_pages[CONFIG_NUM_TABS];
    int              current_tab;
} ConfigPageState;

static void on_tab_switched(GtkNotebook *nb, GtkWidget *page,
                            guint page_num, gpointer data)
{
    (void)nb; (void)page;
    ConfigPageState *st = (ConfigPageState *)data;

    /* Hide old tab */
    if (st->current_tab >= 0 && st->current_tab < CONFIG_NUM_TABS) {
        if (st->tab_pages[st->current_tab]->on_hide)
            st->tab_pages[st->current_tab]->on_hide(st->app);
    }

    st->current_tab = (int)page_num;

    /* Show new tab */
    if (st->current_tab >= 0 && st->current_tab < CONFIG_NUM_TABS) {
        if (st->tab_pages[st->current_tab]->on_show)
            st->tab_pages[st->current_tab]->on_show(st->app);
    }
}

static GtkWidget *config_page_build(AppCtx *app)
{
    ConfigPageState *st = (ConfigPageState *)calloc(1, sizeof(ConfigPageState));
    st->app = app;
    st->current_tab = -1;
    app->page_config_state = st;

    st->notebook = gtk_notebook_new();
    gtk_notebook_set_tab_pos(GTK_NOTEBOOK(st->notebook), GTK_POS_TOP);

    /* Tab order: Camera, Motion, BirdView, Sensor, DVR, Events */
    st->tab_pages[0] = &page_camera_config;
    st->tab_pages[1] = &page_motion_config;
    st->tab_pages[2] = &page_birdview_config;
    st->tab_pages[3] = &page_sensor_config;
    st->tab_pages[4] = &page_dvr_config;
    st->tab_pages[5] = &page_events;
    st->tab_pages[6] = &page_lens_config;

    for (int i = 0; i < CONFIG_NUM_TABS; i++) {
        GtkWidget *content = st->tab_pages[i]->build(app);
        GtkWidget *label = gtk_label_new(st->tab_pages[i]->title);
        gtk_notebook_append_page(GTK_NOTEBOOK(st->notebook), content, label);
    }

    g_signal_connect(st->notebook, "switch-page",
                     G_CALLBACK(on_tab_switched), st);

    return st->notebook;
}

static void config_page_on_show(AppCtx *app)
{
    ConfigPageState *st = (ConfigPageState *)app->page_config_state;
    if (!st) return;

    int cur = gtk_notebook_get_current_page(GTK_NOTEBOOK(st->notebook));
    st->current_tab = cur;
    if (cur >= 0 && cur < CONFIG_NUM_TABS) {
        if (st->tab_pages[cur]->on_show)
            st->tab_pages[cur]->on_show(app);
    }
}

static void config_page_on_hide(AppCtx *app)
{
    ConfigPageState *st = (ConfigPageState *)app->page_config_state;
    if (!st) return;

    if (st->current_tab >= 0 && st->current_tab < CONFIG_NUM_TABS) {
        if (st->tab_pages[st->current_tab]->on_hide)
            st->tab_pages[st->current_tab]->on_hide(app);
    }
}

static void config_page_cleanup(AppCtx *app)
{
    ConfigPageState *st = (ConfigPageState *)app->page_config_state;
    if (!st) return;

    for (int i = 0; i < CONFIG_NUM_TABS; i++) {
        if (st->tab_pages[i]->cleanup)
            st->tab_pages[i]->cleanup(app);
    }

    free(st);
    app->page_config_state = NULL;
}

const PageDef page_config = {
    .id      = "config",
    .title   = "Configuration",
    .build   = config_page_build,
    .on_show = config_page_on_show,
    .on_hide = config_page_on_hide,
    .cleanup = config_page_cleanup,
};
