#include <gtk/gtk.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#define NUM_BUTTONS 20
#define CONFIG_FILE "config.txt"

GtkWidget *text_view;
GtkWidget *buttons[NUM_BUTTONS];
char *button_messages[NUM_BUTTONS];
char serial_port[256] = "/dev/ttyUSB0";
char tempStr [256];
speed_t baud_rate = B9600;
int serial_fd = -1;
pthread_t reader_thread;
gboolean keep_reading = TRUE;

GtkComboBoxText *port_selector;
GtkComboBoxText *baud_selector;

const char *baud_options[] = {"9600", "19200", "38400", "57600", "115200", NULL};
const speed_t baud_constants[] = {B9600, B19200, B38400, B57600, B115200};

void log_message(const char *message, const char *color) {
    GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(text_view));
	
	GtkTextIter end;
    gtk_text_buffer_get_end_iter(buffer, &end);

    char formatted[1024];
    snprintf(formatted, sizeof(formatted), "<span foreground=\"%s\">%s</span>\n", color, message);
    gtk_text_buffer_insert_markup(buffer, &end, formatted, -1);
	
	GtkTextMark *endMark = gtk_text_buffer_create_mark (buffer, "end", &end, 0);
	gtk_text_view_scroll_to_mark (GTK_TEXT_VIEW(text_view), endMark,0,0,0,0);
}

void logGreenMessage( char * message) {
    log_message( message, "green");
    // free the memory that was allocated using strdup
    free(message);
}

void *serial_reader(void *arg) {
    char buf[256];
    
    while (keep_reading && serial_fd >= 0) {
        int n = read(serial_fd, buf, 255);
        if (n > 0) {
            buf[n] = '\0';
	    sprintf(tempStr, "RECV: %s", buf);
	    // strdup creates a copy of the string on the heap so we must free it later
	    g_idle_add_once((GSourceOnceFunc)logGreenMessage, g_strdup(tempStr));	    
        }
    }
    return NULL;
}

void send_serial_message(const char *msg) {
    if (serial_fd >= 0) {
        write(serial_fd, msg, strlen(msg));
        write(serial_fd, "\n", 1);
        char log_line[512];
        snprintf(log_line, sizeof(log_line), "SENT: %s", msg);
        log_message(log_line, "blue");
    } else {
        log_message("Serial port not open.", "red");
    }
}

void on_button_right_click(GtkWidget *widget, GdkEventButton *event, gpointer data) {		
    if (event->type == GDK_BUTTON_PRESS) {
		int index = GPOINTER_TO_INT(data);
		switch( event->button ){
		case 3:
			GtkWidget *dialog = gtk_dialog_new_with_buttons("Edit Button Label",
				GTK_WINDOW(gtk_widget_get_toplevel(widget)),
				GTK_DIALOG_MODAL,
				"_OK", GTK_RESPONSE_OK,
				"_Cancel", GTK_RESPONSE_CANCEL,
				NULL);
			GtkWidget *content = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
			GtkWidget *entry = gtk_entry_new();
			gtk_entry_set_text(GTK_ENTRY(entry), button_messages[index]);
			gtk_box_pack_start(GTK_BOX(content), entry, FALSE, FALSE, 5);
			gtk_widget_show_all(dialog);
			if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_OK) {
				const char *new_label = gtk_entry_get_text(GTK_ENTRY(entry));
				free(button_messages[index]);
				button_messages[index] = strdup(new_label);
				gtk_button_set_label(GTK_BUTTON(buttons[index]), new_label);
			}
			gtk_widget_destroy(dialog);
			break;
		case 1:
			send_serial_message(button_messages[index]);
			break;
		default:
			break;
		}
    }
}

void open_serial_port(GtkWidget *button) {
    if (serial_fd >= 0) {
        close(serial_fd);
        keep_reading = FALSE;
        pthread_cancel(reader_thread);
		log_message("Serial port closed.", "green");
		gtk_button_set_label(GTK_BUTTON(button), "Open port");
    }
	else {
		const char *port = gtk_combo_box_text_get_active_text(port_selector);
		strcpy(serial_port, port);

		const char *baud_str = gtk_combo_box_text_get_active_text(baud_selector);
		for (int i = 0; baud_options[i]; i++) {
			if (strcmp(baud_str, baud_options[i]) == 0) {
				baud_rate = baud_constants[i];
				break;
			}
		}

		serial_fd = open(serial_port, O_RDWR | O_NOCTTY);
		if (serial_fd < 0) {
			log_message("Failed to open serial port.", "red");
			return;
		}

		struct termios tty;
		tcgetattr(serial_fd, &tty);
		cfsetospeed(&tty, baud_rate);
		cfsetispeed(&tty, baud_rate);
		cfmakeraw(&tty);	
		tty.c_cc[VMIN] = 30;
		tty.c_cc[VTIME] = 1;		
		tcsetattr(serial_fd, TCSANOW, &tty);

		keep_reading = TRUE;
		pthread_create(&reader_thread, NULL, serial_reader, NULL);
		log_message("Serial port opened.", "green");
		gtk_button_set_label(GTK_BUTTON(button), "Close port");
	}
}

void load_config() {
    FILE *fp = fopen(CONFIG_FILE, "r");
    if (!fp) return;
    char line[512];
    for (int i = 0; i < NUM_BUTTONS && fgets(line, sizeof(line), fp); ++i) {
        line[strcspn(line, "\n\r")] = 0;
        free(button_messages[i]);
        button_messages[i] = strdup(line);
        gtk_button_set_label(GTK_BUTTON(buttons[i]), line);
    }
    fclose(fp);
}

void reload_config(GtkWidget *button) {
	load_config();
	log_message("Configuration loaded", "red");
}

void save_config() {
    FILE *fp = fopen(CONFIG_FILE, "w");
    if (!fp) return;
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        fprintf(fp, "%s\n", button_messages[i]);
    }
    fclose(fp);
    log_message("Configuration saved.", "black");
}

void list_serial_ports() {
    DIR *d = opendir("/dev");
    struct dirent *dir;
    while ((dir = readdir(d)) != NULL) {
        if (strncmp(dir->d_name, "ttyUSB", 6) == 0 || strncmp(dir->d_name, "ttyACM", 6) == 0) {
            char fullpath[256];
            snprintf(fullpath, sizeof(fullpath), "/dev/%s", dir->d_name);
            gtk_combo_box_text_append_text(port_selector, fullpath);
        }
    }
    closedir(d);
    gtk_combo_box_set_active(GTK_COMBO_BOX(port_selector), 0);
}

void on_destroy() {
    keep_reading = FALSE;
    if (serial_fd >= 0) close(serial_fd);
    pthread_cancel(reader_thread);
    save_config();
    gtk_main_quit();
}

// Function to populate the port combo box
void populate_serial_ports(GtkComboBoxText *combo_box) {
    DIR *dev_dir = opendir("/dev");
    if (!dev_dir) return;

    struct dirent *entry;
    while ((entry = readdir(dev_dir)) != NULL) {
        if (strncmp(entry->d_name, "ttyUSB", 6) == 0 ||
            strncmp(entry->d_name, "ttyAMA", 6) == 0 ||
            strncmp(entry->d_name, "ttyS", 4) == 0 ||
            strncmp(entry->d_name, "serial", 6) == 0) {

            char full_path[64];
            snprintf(full_path, sizeof(full_path), "/dev/%s", entry->d_name);
            gtk_combo_box_text_append_text(combo_box, full_path);
        }
    }
    closedir(dev_dir);
}

int main(int argc, char *argv[]) {
    gtk_init(&argc, &argv);

    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Serial Button Panel");
    gtk_window_set_default_size(GTK_WINDOW(window), 800, 600);
    g_signal_connect(window, "destroy", G_CALLBACK(on_destroy), NULL);

    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_add(GTK_CONTAINER(window), vbox);

    // Serial config
    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    port_selector = GTK_COMBO_BOX_TEXT(gtk_combo_box_text_new());
	populate_serial_ports(port_selector);
	gtk_combo_box_set_active(GTK_COMBO_BOX(port_selector), 0);
	
	
    baud_selector = GTK_COMBO_BOX_TEXT(gtk_combo_box_text_new());

    gtk_box_pack_start(GTK_BOX(hbox), gtk_label_new("Port:"), FALSE, FALSE, 2);
    gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(port_selector), FALSE, FALSE, 2);
    gtk_box_pack_start(GTK_BOX(hbox), gtk_label_new("Baud:"), FALSE, FALSE, 2);
    gtk_box_pack_start(GTK_BOX(hbox), GTK_WIDGET(baud_selector), FALSE, FALSE, 2);

    GtkWidget *connect_button = gtk_button_new_with_label("Open Port");
    g_signal_connect(connect_button, "clicked", G_CALLBACK(open_serial_port), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), connect_button, FALSE, FALSE, 2);
	
	GtkWidget *reload_button = gtk_button_new_with_label("Reload");
    g_signal_connect(reload_button, "clicked", G_CALLBACK(load_config), NULL);
    gtk_box_pack_start(GTK_BOX(hbox), reload_button, FALSE, FALSE, 2);
	
    gtk_box_pack_start(GTK_BOX(vbox), hbox, FALSE, FALSE, 2);

    for (int i = 0; baud_options[i]; ++i)
        gtk_combo_box_text_append_text(baud_selector, baud_options[i]);
    gtk_combo_box_set_active(GTK_COMBO_BOX(baud_selector), 0);

    list_serial_ports();

    GtkWidget *grid = gtk_grid_new();
	gtk_grid_set_column_spacing( GTK_GRID(grid), 10);
	gtk_grid_set_row_spacing( GTK_GRID(grid), 10);
    gtk_box_pack_start(GTK_BOX(vbox), grid, FALSE, FALSE, 5);

    for (int i = 0; i < NUM_BUTTONS; ++i) {
        buttons[i] = gtk_button_new_with_label("Message");
        button_messages[i] = strdup("Message");
//        g_signal_connect(buttons[i], "clicked", G_CALLBACK(on_button_clicked), GINT_TO_POINTER(i));
        g_signal_connect(buttons[i], "button-press-event", G_CALLBACK(on_button_right_click), GINT_TO_POINTER(i));
        gtk_grid_attach(GTK_GRID(grid), buttons[i], i % 5, i / 5, 1, 1);
    }

    text_view = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(text_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(text_view), GTK_WRAP_WORD_CHAR);
    GtkWidget *scroll = gtk_scrolled_window_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(scroll), text_view);
    gtk_box_pack_start(GTK_BOX(vbox), scroll, TRUE, TRUE, 5);

    load_config();
    gtk_widget_show_all(window);
    gtk_main();
    return 0;
}
