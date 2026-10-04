"""Durable family and wardrobe records. No cloud inference or actuator access."""
import base64
import hashlib
import json
import os
import sqlite3
import time
import uuid
from contextlib import contextmanager


class FamilyStore:
    def __init__(self, path):
        self.path = os.path.abspath(path)
        os.makedirs(os.path.dirname(self.path), mode=0o700, exist_ok=True)
        with self.connect() as db:
            version = db.execute('PRAGMA user_version').fetchone()[0]
            if version not in (0, 1, 2):
                raise ValueError('Unsupported family database version')
            db.executescript('''
              CREATE TABLE IF NOT EXISTS profiles(id TEXT PRIMARY KEY, label TEXT NOT NULL,
                revision TEXT NOT NULL, payload TEXT NOT NULL);
              CREATE TABLE IF NOT EXISTS settings(key TEXT PRIMARY KEY, value TEXT);
              CREATE TABLE IF NOT EXISTS outfits(id TEXT PRIMARY KEY, profile_id TEXT NOT NULL
                REFERENCES profiles(id) ON DELETE CASCADE, attributes TEXT NOT NULL,
                descriptors TEXT NOT NULL, anchor TEXT NOT NULL, created REAL NOT NULL);
              CREATE TABLE IF NOT EXISTS views(outfit_id TEXT REFERENCES outfits(id) ON DELETE CASCADE,
                digest TEXT NOT NULL, jpeg BLOB NOT NULL, created REAL NOT NULL,
                PRIMARY KEY(outfit_id,digest));
            ''')
            db.execute('BEGIN IMMEDIATE')
            columns={r['name'] for r in db.execute('PRAGMA table_info(views)')}
            if 'appearance' not in columns:
                db.execute("ALTER TABLE views ADD COLUMN appearance TEXT NOT NULL DEFAULT '{}'")
            if 'visual_hash' not in columns:
                db.execute("ALTER TABLE views ADD COLUMN visual_hash TEXT")
            db.execute('PRAGMA user_version=2')
        os.chmod(self.path, 0o600)

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=2)
        db.row_factory = sqlite3.Row
        db.execute('PRAGMA foreign_keys=ON')
        db.execute('PRAGMA journal_mode=WAL')
        try:
            with db:
                yield db
        finally:
            db.close()

    def migrate(self, legacy):
        """Exactly once, including an empty installation; never modify the old file."""
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            if db.execute("SELECT 1 FROM settings WHERE key='legacy_imported'").fetchone():
                return
            if legacy:
                pid = uuid.uuid4().hex
                db.execute('INSERT INTO profiles VALUES(?,?,?,?)',
                           (pid, legacy['label'], uuid.uuid4().hex, json.dumps(legacy)))
                db.execute("INSERT OR REPLACE INTO settings VALUES('selected',?)", (pid,))
            db.execute("INSERT INTO settings VALUES('legacy_imported','1')")

    def profiles(self):
        with self.connect() as db:
            return [dict(id=r['id'], label=r['label'], revision=r['revision'],
                         sample_count=len(json.loads(r['payload'])['samples']))
                    for r in db.execute('SELECT * FROM profiles ORDER BY rowid')]

    def selected_id(self):
        with self.connect() as db:
            row = db.execute("SELECT value FROM settings WHERE key='selected'").fetchone()
            return row['value'] if row else None

    def load(self, profile_id=None):
        pid = profile_id or self.selected_id()
        with self.connect() as db:
            row = db.execute('SELECT * FROM profiles WHERE id=?', (pid,)).fetchone()
            if not row:
                return None
            return dict(json.loads(row['payload']), label=row['label'], profile_id=row['id'],
                        created_utc=row['revision'], revision=row['revision'])

    def save_profile(self, payload, profile_id=None):
        label = str(payload.get('label', '')).strip()
        if not label or len(label) > 40 or not payload.get('samples'):
            raise ValueError('A name and enrolled face samples are required')
        pid, revision = profile_id or uuid.uuid4().hex, uuid.uuid4().hex
        with self.connect() as db:
            if profile_id and not db.execute('SELECT 1 FROM profiles WHERE id=?', (pid,)).fetchone():
                raise ValueError('Profile no longer exists')
            db.execute('INSERT INTO profiles VALUES(?,?,?,?) ON CONFLICT(id) DO UPDATE SET '
                       'label=excluded.label, revision=excluded.revision, payload=excluded.payload',
                       (pid, label, revision, json.dumps(payload)))
            db.execute("INSERT OR REPLACE INTO settings VALUES('selected',?)", (pid,))
        return self.load(pid)

    def select(self, pid):
        with self.connect() as db:
            if not db.execute('SELECT 1 FROM profiles WHERE id=?', (pid,)).fetchone():
                raise ValueError('Unknown family member')
            db.execute("INSERT OR REPLACE INTO settings VALUES('selected',?)", (pid,))

    def rename(self, pid, label):
        label = str(label).strip()
        if not label or len(label) > 40:
            raise ValueError('Name must be 1–40 characters')
        with self.connect() as db:
            if not db.execute('UPDATE profiles SET label=? WHERE id=?', (label, pid)).rowcount:
                raise ValueError('Unknown family member')

    def delete(self, pid):
        with self.connect() as db:
            db.execute('DELETE FROM profiles WHERE id=?', (pid,))
            if self.selected_id() == pid:
                row = db.execute('SELECT id FROM profiles ORDER BY rowid LIMIT 1').fetchone()
                db.execute("INSERT OR REPLACE INTO settings VALUES('selected',?)", (row['id'] if row else None,))

    def remember(self, pid, revision, attributes, descriptors, jpeg, anchor, outfit_id=None):
        if anchor.get('source') != 'face' or not anchor.get('confirmed') or not anchor.get('frame_key'):
            raise ValueError('Permanent clothing needs a confirmed face/body observation')
        if not isinstance(jpeg, bytes) or not 100 <= len(jpeg) <= 500_000:
            raise ValueError('Invalid clothing crop')
        oid = outfit_id or uuid.uuid4().hex
        with self.connect() as db:
            p = db.execute('SELECT revision FROM profiles WHERE id=?', (pid,)).fetchone()
            if not p or p['revision'] != revision:
                raise ValueError('Profile changed while observing clothes')
            if outfit_id:
                if not db.execute('SELECT 1 FROM outfits WHERE id=? AND profile_id=?', (oid,pid)).fetchone():
                    raise ValueError('Clothing record belongs to another profile')
            else:
                db.execute('INSERT INTO outfits VALUES(?,?,?,?,?,?)',
                           (oid,pid,json.dumps(attributes),json.dumps(descriptors),json.dumps(anchor),time.time()))
            visual_hash=clothing_visual_hash(jpeg)
            existing=db.execute('SELECT digest,visual_hash,created FROM views WHERE outfit_id=? ORDER BY created', (oid,)).fetchall()
            # Refresh a redundant view, retaining distinct angles. If all six
            # differ, evict the older member of the closest visual pair.
            distance=lambda a,b:bin(int(a,16)^int(b,16)).count('1')
            close=[r for r in existing if visual_hash and r['visual_hash'] and distance(visual_hash,r['visual_hash'])<=5]
            if close:
                replace=min(close,key=lambda r:distance(visual_hash,r['visual_hash']))
                db.execute('DELETE FROM views WHERE outfit_id=? AND digest=?',(oid,replace['digest']))
            db.execute('INSERT OR IGNORE INTO views(outfit_id,digest,jpeg,created,appearance,visual_hash) VALUES(?,?,?,?,?,?)',
                       (oid, hashlib.sha256(jpeg).hexdigest(), jpeg, time.time(),json.dumps(descriptors),visual_hash))
            views=db.execute('SELECT digest,visual_hash,created FROM views WHERE outfit_id=? ORDER BY created', (oid,)).fetchall()
            if len(views)>6:
                pairs=[(distance(a['visual_hash'],b['visual_hash']),i,j) for i,a in enumerate(views) for j,b in enumerate(views)
                       if i<j and a['visual_hash'] and b['visual_hash']]
                remove=views[min(pairs)[1]] if pairs else views[0]
                db.execute('DELETE FROM views WHERE outfit_id=? AND digest=?',(oid,remove['digest']))
        return oid

    def wardrobe(self, pid=None, images=False):
        with self.connect() as db:
            query = 'SELECT o.*,p.revision FROM outfits o JOIN profiles p ON p.id=o.profile_id'
            rows = db.execute(query + (' WHERE profile_id=?' if pid else '') + ' ORDER BY created DESC',
                              (pid,) if pid else ())
            result = []
            for r in rows:
                item = {k:r[k] for k in ('id','profile_id','revision','created')}
                item.update({k:json.loads(r[k]) for k in ('attributes','descriptors','anchor')})
                views = db.execute('SELECT jpeg,appearance FROM views WHERE outfit_id=? ORDER BY created DESC', (r['id'],)).fetchall()
                item['view_count'] = len(views)
                if images:
                    item['images'] = [base64.b64encode(v['jpeg']).decode() for v in views]
                    item['views'] = [dict(image=image,descriptors=json.loads(v['appearance']) or item['descriptors'])
                                     for image,v in zip(item['images'],views)]
                result.append(item)
            return result

    def forget(self, pid, oid):
        with self.connect() as db:
            db.execute('DELETE FROM outfits WHERE id=? AND profile_id=?', (oid,pid))


def clothing_visual_hash(jpeg):
    try:
        import cv2
        import numpy as np
        image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_GRAYSCALE)
        if image is None:return None
        small=cv2.resize(image,(9,8))
        bits=(small[:,1:]>small[:,:-1]).ravel()
        return format(sum(int(bit)<<i for i,bit in enumerate(bits)),'016x')
    except ImportError:
        return None


class FamilyTargetStore:
    """Selected-profile adapter for existing enrollment and face services."""
    def __init__(self, legacy):
        self.legacy = legacy
        self.path = legacy.path
        self.model_id, self.model_sha256 = legacy.model_id, legacy.model_sha256
        self.family = FamilyStore(os.path.join(os.path.dirname(self.path), 'people.sqlite3'))
        # Only parse the old store if migration is actually needed.
        with self.family.connect() as db:
            migrated = db.execute("SELECT 1 FROM settings WHERE key='legacy_imported'").fetchone()
        if not migrated:
            self.family.migrate(legacy.load())

    def load(self):
        result = self.family.load()
        if result and result.get('model') != {'id':self.model_id,'sha256':self.model_sha256}:
            raise ValueError('Profile was enrolled with a different face model')
        return result

    def save(self, label, samples, retention, profile_id=None):
        return self.family.save_profile(dict(label=label,samples=samples,retention=retention,
            model=dict(id=self.model_id,sha256=self.model_sha256)), profile_id)

    def delete(self):
        self.family.delete(self.family.selected_id())
