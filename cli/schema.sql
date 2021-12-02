

PRAGMA journal_mode = WAL;
PRAGMA foreign_keys = ON;


CREATE TABLE IF NOT EXISTS joints (
        insertedat DATETIME DEFAULT (STRFTIME('%Y-%m-%d %H:%M:%f', 'NOW', 'localtime')),
        J1 REAL NOT NULL,
        J2 REAL NOT NULL,
        J3 REAL NOT NULL,
        J4 REAL NOT NULL,
        J5 REAL NOT NULL,
        J6 REAL NOT NULL
);
