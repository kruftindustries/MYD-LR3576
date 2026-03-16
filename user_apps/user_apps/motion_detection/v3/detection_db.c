/*
 * detection_db.c - SQLite storage for YOLO detection events
 */

#include "detection_db.h"

#include <stdio.h>
#include <string.h>

static const char *CREATE_SQL =
	"CREATE TABLE IF NOT EXISTS detections ("
	"  id INTEGER PRIMARY KEY AUTOINCREMENT,"
	"  timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%f','now','localtime')),"
	"  stream_id INTEGER NOT NULL,"
	"  class_name TEXT NOT NULL,"
	"  confidence REAL NOT NULL,"
	"  box_left INTEGER,"
	"  box_top INTEGER,"
	"  box_right INTEGER,"
	"  box_bottom INTEGER,"
	"  motion_pct REAL"
	");";

sqlite3 *db_open(const char *path)
{
	sqlite3 *db = NULL;
	int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX;

	int rc = sqlite3_open_v2(path, &db, flags, NULL);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "DB open failed: %s\n", sqlite3_errmsg(db));
		if (db)
			sqlite3_close(db);
		return NULL;
	}

	/* Enable WAL for concurrent read/write */
	char *err = NULL;
	sqlite3_exec(db, "PRAGMA journal_mode=WAL;", NULL, NULL, &err);
	if (err) {
		fprintf(stderr, "DB WAL warning: %s\n", err);
		sqlite3_free(err);
	}

	/* Create table */
	err = NULL;
	rc = sqlite3_exec(db, CREATE_SQL, NULL, NULL, &err);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "DB create table failed: %s\n", err);
		sqlite3_free(err);
		sqlite3_close(db);
		return NULL;
	}

	return db;
}

int db_insert(sqlite3 *db, int stream_id, const char *class_name,
              float confidence, int x1, int y1, int x2, int y2,
              float motion_pct)
{
	static const char *INSERT_SQL =
		"INSERT INTO detections "
		"(stream_id, class_name, confidence, box_left, box_top, box_right, box_bottom, motion_pct) "
		"VALUES (?, ?, ?, ?, ?, ?, ?, ?);";

	sqlite3_stmt *stmt = NULL;
	int rc = sqlite3_prepare_v2(db, INSERT_SQL, -1, &stmt, NULL);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "DB prepare failed: %s\n", sqlite3_errmsg(db));
		return -1;
	}

	sqlite3_bind_int(stmt, 1, stream_id);
	sqlite3_bind_text(stmt, 2, class_name, -1, SQLITE_TRANSIENT);
	sqlite3_bind_double(stmt, 3, (double)confidence);
	sqlite3_bind_int(stmt, 4, x1);
	sqlite3_bind_int(stmt, 5, y1);
	sqlite3_bind_int(stmt, 6, x2);
	sqlite3_bind_int(stmt, 7, y2);
	sqlite3_bind_double(stmt, 8, (double)motion_pct);

	rc = sqlite3_step(stmt);
	sqlite3_finalize(stmt);

	if (rc != SQLITE_DONE) {
		fprintf(stderr, "DB insert failed: %s\n", sqlite3_errmsg(db));
		return -1;
	}

	return 0;
}

void db_close(sqlite3 *db)
{
	if (db)
		sqlite3_close(db);
}
