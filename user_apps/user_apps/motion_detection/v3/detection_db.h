#ifndef DETECTION_DB_H
#define DETECTION_DB_H

#include <sqlite3.h>

/*
 * Open (or create) the detections database.
 * Creates the detections table if it doesn't exist.
 * Enables WAL journal mode and FULLMUTEX for thread safety.
 * Returns NULL on failure.
 */
sqlite3 *db_open(const char *path);

/*
 * Insert a detection event.
 * Timestamp is generated automatically (local time, millisecond precision).
 * Returns 0 on success, -1 on failure.
 */
int db_insert(sqlite3 *db, int stream_id, const char *class_name,
              float confidence, int x1, int y1, int x2, int y2,
              float motion_pct);

/*
 * Close the database connection.
 */
void db_close(sqlite3 *db);

#endif /* DETECTION_DB_H */
