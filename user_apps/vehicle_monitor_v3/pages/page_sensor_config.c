/*
 * pages/page_sensor_config.c - RS485/CAN sensor configuration
 *
 * Add/edit/delete sensors. I/O is stubbed (simulated random-walk).
 * Config stored in SQLite sensors table.
 */
#include "../app.h"
#include "../core/database.h"
#include "../core/config.h"
#include "../ui/util.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    AppCtx     *app;
    GtkWidget  *list_box;
    GtkWidget  *form_box;
    GtkWidget  *btn_add;
    GtkWidget  *btn_delete;
    GtkWidget  *btn_save;

    /* Form fields */
    GtkWidget  *ent_name;
    GtkWidget  *cmb_type;
    GtkWidget  *ent_address;
    GtkWidget  *spn_register;
    GtkWidget  *spn_scale;
    GtkWidget  *spn_offset;
    GtkWidget  *ent_units;
    GtkWidget  *spn_poll;
    GtkWidget  *chk_enabled;

    int         selected_id;
} SensorConfigState;

static void populate_list(SensorConfigState *st);
static void load_form(SensorConfigState *st, int sensor_id);

static void on_list_row_selected(GtkListBox *box, GtkListBoxRow *row,
                                 gpointer data)
{
    (void)box;
    SensorConfigState *st = (SensorConfigState *)data;
    if (!row) {
        st->selected_id = 0;
        gtk_widget_set_sensitive(st->form_box, FALSE);
        gtk_widget_set_sensitive(st->btn_delete, FALSE);
        return;
    }
    int id = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(row), "sensor-id"));
    load_form(st, id);
    gtk_widget_set_sensitive(st->form_box, TRUE);
    gtk_widget_set_sensitive(st->btn_delete, TRUE);
}

static void on_add_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    SensorConfigState *st = (SensorConfigState *)data;
    DbSensor s;
    memset(&s, 0, sizeof(s));
    snprintf(s.name, sizeof(s.name), "Sensor %d",
             st->app->num_sensors + 1);
    snprintf(s.type, sizeof(s.type), "RS485");
    snprintf(s.address, sizeof(s.address), "1");
    s.scale = 1.0;
    s.offset = 0.0;
    s.poll_interval_ms = 1000;
    s.enabled = 1;

    int id = db_sensor_insert(st->app->db, &s);
    if (id > 0) {
        config_reload_sensors(st->app);
        populate_list(st);
        load_form(st, id);
    }
}

static void on_delete_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    SensorConfigState *st = (SensorConfigState *)data;
    if (st->selected_id <= 0) return;

    GtkWidget *dlg = gtk_message_dialog_new(
        GTK_WINDOW(st->app->window),
        GTK_DIALOG_MODAL, GTK_MESSAGE_QUESTION, GTK_BUTTONS_YES_NO,
        "Delete sensor %d?", st->selected_id);
    int resp = gtk_dialog_run(GTK_DIALOG(dlg));
    gtk_widget_destroy(dlg);
    if (resp != GTK_RESPONSE_YES) return;

    /* Stop sensor thread if running */
    for (int i = 0; i < st->app->num_sensors; i++) {
        if (st->app->sensors[i].id == st->selected_id) {
            st->app->sensors[i].running = 0;
            if (st->app->sensors[i].enabled)
                pthread_join(st->app->sensors[i].thread, NULL);
            /* Shift remaining */
            for (int j = i; j < st->app->num_sensors - 1; j++)
                st->app->sensors[j] = st->app->sensors[j + 1];
            st->app->num_sensors--;
            memset(&st->app->sensors[st->app->num_sensors], 0,
                   sizeof(SensorCtx));
            break;
        }
    }

    db_sensor_delete(st->app->db, st->selected_id);
    st->selected_id = 0;
    populate_list(st);
    gtk_widget_set_sensitive(st->form_box, FALSE);
    gtk_widget_set_sensitive(st->btn_delete, FALSE);
}

static void on_save_clicked(GtkButton *btn, gpointer data)
{
    (void)btn;
    SensorConfigState *st = (SensorConfigState *)data;
    if (st->selected_id <= 0) return;

    DbSensor s;
    s.id = st->selected_id;
    snprintf(s.name, sizeof(s.name), "%s",
             gtk_entry_get_text(GTK_ENTRY(st->ent_name)));

    int type_idx = gtk_combo_box_get_active(GTK_COMBO_BOX(st->cmb_type));
    snprintf(s.type, sizeof(s.type), "%s",
             type_idx == 0 ? "RS485" : "CAN");

    snprintf(s.address, sizeof(s.address), "%s",
             gtk_entry_get_text(GTK_ENTRY(st->ent_address)));
    s.register_addr = gtk_spin_button_get_value_as_int(
        GTK_SPIN_BUTTON(st->spn_register));
    s.scale = gtk_spin_button_get_value(GTK_SPIN_BUTTON(st->spn_scale));
    s.offset = gtk_spin_button_get_value(GTK_SPIN_BUTTON(st->spn_offset));
    snprintf(s.units, sizeof(s.units), "%s",
             gtk_entry_get_text(GTK_ENTRY(st->ent_units)));
    s.poll_interval_ms = gtk_spin_button_get_value_as_int(
        GTK_SPIN_BUTTON(st->spn_poll));
    s.enabled = gtk_toggle_button_get_active(
        GTK_TOGGLE_BUTTON(st->chk_enabled)) ? 1 : 0;

    db_sensor_update(st->app->db, &s);
    config_reload_sensors(st->app);
    populate_list(st);

    printf("Sensor %d saved: %s\n", s.id, s.name);
}

static void populate_list(SensorConfigState *st)
{
    GList *children = gtk_container_get_children(
        GTK_CONTAINER(st->list_box));
    for (GList *l = children; l; l = l->next)
        gtk_container_remove(GTK_CONTAINER(st->list_box),
                             GTK_WIDGET(l->data));
    g_list_free(children);

    DbSensor sensors[MAX_SENSORS];
    int n = db_sensor_load_all(st->app->db, sensors, MAX_SENSORS);

    for (int i = 0; i < n; i++) {
        char label[128];
        snprintf(label, sizeof(label), "%s (%s)%s",
                 sensors[i].name, sensors[i].type,
                 sensors[i].enabled ? "" : " [off]");
        GtkWidget *row = gtk_list_box_row_new();
        GtkWidget *lbl = gtk_label_new(label);
        gtk_label_set_xalign(GTK_LABEL(lbl), 0.0);
        gtk_widget_set_margin_start(lbl, 8);
        gtk_widget_set_margin_top(lbl, 4);
        gtk_widget_set_margin_bottom(lbl, 4);
        gtk_container_add(GTK_CONTAINER(row), lbl);
        g_object_set_data(G_OBJECT(row), "sensor-id",
                          GINT_TO_POINTER(sensors[i].id));
        gtk_list_box_insert(GTK_LIST_BOX(st->list_box), row, -1);
    }

    gtk_widget_show_all(st->list_box);
}

static void load_form(SensorConfigState *st, int sensor_id)
{
    st->selected_id = sensor_id;

    DbSensor sensors[MAX_SENSORS];
    int n = db_sensor_load_all(st->app->db, sensors, MAX_SENSORS);

    for (int i = 0; i < n; i++) {
        if (sensors[i].id != sensor_id) continue;
        DbSensor *s = &sensors[i];

        gtk_entry_set_text(GTK_ENTRY(st->ent_name), s->name);
        gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_type),
                                 strcmp(s->type, "CAN") == 0 ? 1 : 0);
        gtk_entry_set_text(GTK_ENTRY(st->ent_address), s->address);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_register),
                                  s->register_addr);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_scale), s->scale);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_offset), s->offset);
        gtk_entry_set_text(GTK_ENTRY(st->ent_units), s->units);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(st->spn_poll),
                                  s->poll_interval_ms);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st->chk_enabled),
                                     s->enabled);
        break;
    }
}

static GtkWidget *sensor_config_build(AppCtx *app)
{
    SensorConfigState *st = (SensorConfigState *)calloc(1,
                                sizeof(SensorConfigState));
    st->app = app;
    app->page_sensor_config_state = st;

    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_widget_set_margin_start(hbox, 8);
    gtk_widget_set_margin_end(hbox, 8);
    gtk_widget_set_margin_top(hbox, 8);
    gtk_widget_set_margin_bottom(hbox, 8);

    /* Left: sensor list + buttons */
    GtkWidget *left = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_size_request(left, 200, -1);

    st->list_box = gtk_list_box_new();
    gtk_list_box_set_selection_mode(GTK_LIST_BOX(st->list_box),
                                   GTK_SELECTION_SINGLE);
    g_signal_connect(st->list_box, "row-selected",
                     G_CALLBACK(on_list_row_selected), st);

    GtkWidget *scroll_list = ui_scrolled(st->list_box);
    gtk_widget_set_vexpand(scroll_list, TRUE);
    gtk_box_pack_start(GTK_BOX(left), scroll_list, TRUE, TRUE, 0);

    GtkWidget *btn_bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    st->btn_add = ui_button("Add", G_CALLBACK(on_add_clicked), st);
    st->btn_delete = ui_button("Delete", G_CALLBACK(on_delete_clicked), st);
    gtk_widget_set_sensitive(st->btn_delete, FALSE);
    gtk_box_pack_start(GTK_BOX(btn_bar), st->btn_add, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(btn_bar), st->btn_delete, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(left), btn_bar, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(hbox), left, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox),
                       gtk_separator_new(GTK_ORIENTATION_VERTICAL),
                       FALSE, FALSE, 0);

    /* Right: form */
    st->form_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_widget_set_sensitive(st->form_box, FALSE);
    gtk_widget_set_hexpand(st->form_box, TRUE);

    GtkWidget *grid = ui_grid();
    int row = 0;

    st->ent_name = ui_entry("", 63, NULL, NULL);
    row = ui_grid_row(grid, row, "Name", st->ent_name);

    st->cmb_type = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_type), "RS485");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(st->cmb_type), "CAN");
    gtk_combo_box_set_active(GTK_COMBO_BOX(st->cmb_type), 0);
    row = ui_grid_row(grid, row, "Type", st->cmb_type);

    st->ent_address = ui_entry("1", 31, NULL, NULL);
    row = ui_grid_row(grid, row, "Address", st->ent_address);

    st->spn_register = ui_spin_int(0, 0, 65535, 1, NULL, NULL);
    row = ui_grid_row(grid, row, "Register", st->spn_register);

    st->spn_scale = ui_spin_float(1.0, -1000.0, 1000.0, 0.1, 3,
                                  NULL, NULL);
    row = ui_grid_row(grid, row, "Scale", st->spn_scale);

    st->spn_offset = ui_spin_float(0.0, -10000.0, 10000.0, 0.1, 3,
                                   NULL, NULL);
    row = ui_grid_row(grid, row, "Offset", st->spn_offset);

    st->ent_units = ui_entry("", 15, NULL, NULL);
    row = ui_grid_row(grid, row, "Units", st->ent_units);

    st->spn_poll = ui_spin_int(1000, 100, 60000, 100, NULL, NULL);
    row = ui_grid_row(grid, row, "Poll (ms)", st->spn_poll);

    st->chk_enabled = gtk_check_button_new_with_label("Enabled");
    gtk_grid_attach(GTK_GRID(grid), st->chk_enabled, 0, row, 2, 1);
    (void)row;

    GtkWidget *scroll_form = ui_scrolled(grid);
    gtk_box_pack_start(GTK_BOX(st->form_box), scroll_form, TRUE, TRUE, 0);

    st->btn_save = ui_button("Save", G_CALLBACK(on_save_clicked), st);
    gtk_box_pack_start(GTK_BOX(st->form_box), st->btn_save,
                       FALSE, FALSE, 4);

    gtk_box_pack_start(GTK_BOX(hbox), st->form_box, TRUE, TRUE, 0);

    populate_list(st);

    return hbox;
}

static void sensor_config_cleanup(AppCtx *app)
{
    if (app->page_sensor_config_state) {
        free(app->page_sensor_config_state);
        app->page_sensor_config_state = NULL;
    }
}

const PageDef page_sensor_config = {
    .id      = "sensor_config",
    .title   = "Sensor Config",
    .build   = sensor_config_build,
    .on_show = NULL,
    .on_hide = NULL,
    .cleanup = sensor_config_cleanup,
};
