

PRAGMA journal_mode = WAL;
PRAGMA foreign_keys = ON;


CREATE TABLE IF NOT EXISTS pose (
        insertedat DATETIME DEFAULT (STRFTIME('%Y-%m-%d %H:%M:%f', 'NOW', 'localtime')),
        X REAL NOT NULL,
        Y REAL NOT NULL,
        Z REAL NOT NULL,
        QW REAL NOT NULL,
        QX REAL NOT NULL,
        QY REAL NOT NULL,
        QZ REAL NOT NULL
);