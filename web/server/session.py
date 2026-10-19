"""Who is on the page, who controls the arm, and what everybody did.

Everything lives in memory in one Hub, so the server must run as ONE process
(uvicorn --workers 1, the default). Each browser tab is a client with an id it
generates itself (sessionStorage) and a name the user typed once (localStorage).

Control lease: one client at a time is the operator. Every command except STOP
needs the lease and runs one at a time (a second one while the first is still
running is refused as busy). The lease ends when the operator releases it,
closes the page (after RECONNECT_GRACE, so a reload keeps it) or does nothing
for IDLE_TIMEOUT seconds.

Speed is shared too: one value for everybody, changed only by the operator and
capped at SPEED_LIMIT unless the operator explicitly unlocks the higher range.
It stays when the lease changes hands, but the unlock does not: a released lease
puts a speed above the limit back to the limit.

The hub is thread-safe: HTTP routes run in a thread pool and ROS callbacks in
the executor's threads; messages reach the websockets through the event loop.
"""

import asyncio
import collections
import threading
import time

IDLE_TIMEOUT = 60.0       # seconds without a command or a control interaction
RECONNECT_GRACE = 5.0     # a reload within this time keeps the lease and the presence
ACTIVITY_LENGTH = 200
QUEUE_SIZE = 300          # messages waiting for one slow browser before it is skipped
NAME_MAX = 32
SPEED_DEFAULT = 30
SPEED_LIMIT = 80          # above this the operator has to unlock the fast range
SPEED_MAX = 100


class LockedError(RuntimeError):
    """Someone else holds the lease (or nobody, and the client did not take it)."""


class BusyError(RuntimeError):
    """Another command is still running."""


def clean_name(name):
    name = " ".join(str(name or "").split())[:NAME_MAX]
    return name or "Guest"


class Client:
    def __init__(self, cid, name, ip):
        self.id = cid
        self.name = name
        self.ip = ip
        self.page = ""
        self.since = time.time()
        self.connections = 0
        self.left_at = None
        self.queues = []   # one asyncio.Queue per open websocket of this client

    def public(self):
        return {"id": self.id, "name": self.name, "ip": self.ip, "page": self.page,
                "since": self.since, "online": self.connections > 0}


class Hub:
    def __init__(self, idle_timeout=IDLE_TIMEOUT, grace=RECONNECT_GRACE, clock=time.time):
        self.idle_timeout = idle_timeout
        self.grace = grace
        self.clock = clock
        self._lock = threading.RLock()
        self._command_lock = threading.Lock()
        self._loop = None
        self._clients = {}
        self._holder = None
        self._since = None
        self._last_activity = None
        self._activity = collections.deque(maxlen=ACTIVITY_LENGTH)
        self._ui = None   # last control-panel state sent by the operator
        self._speed = SPEED_DEFAULT
        self._fast = False   # the operator accepted the warning for speeds above SPEED_LIMIT

    def bind(self, loop):
        self._loop = loop

    # ---- sending ------------------------------------------------------------------

    def broadcast(self, message, only=None, exclude=None):
        """Queue `message` for every connected tab (or the client ids in `only`). Any thread."""
        if self._loop is None:
            return
        with self._lock:
            queues = [q for c in self._clients.values()
                      if (only is None or c.id in only) and c.id != exclude
                      for q in c.queues]
        if queues:
            self._loop.call_soon_threadsafe(self._fanout, queues, message)

    @staticmethod
    def _fanout(queues, message):
        for q in queues:
            if q.qsize() < QUEUE_SIZE:
                q.put_nowait(message)

    def send_to(self, cid, message):
        self.broadcast(message, only={cid})

    # ---- clients ------------------------------------------------------------------

    def connect(self, cid, name, ip, queue):
        """A websocket said hello. Returns the messages that bring it up to date."""
        name = clean_name(name)
        with self._lock:
            client = self._clients.get(cid)
            returning = client is not None
            if client is None:
                client = self._clients[cid] = Client(cid, name, ip)
            renamed = returning and client.name != name
            client.name, client.ip = name, ip
            client.connections += 1
            client.left_at = None
            client.queues.append(queue)
            welcome = [
                {"type": "welcome", "you": cid, "name": name, "idle_timeout": self.idle_timeout},
                {"type": "activity", "entries": list(self._activity)},
                self._speed_message(),
            ]
            if self._ui is not None:
                welcome.append(dict(self._ui, type="ui"))
        if not returning:
            self.log(name, "joined", ok=True, by=cid)
        elif renamed:
            self.log(name, "renamed", ok=True, by=cid)
        self._send_presence()
        self._send_lease()
        return welcome

    def disconnect(self, cid, queue):
        with self._lock:
            client = self._clients.get(cid)
            if client is None:
                return
            if queue in client.queues:
                client.queues.remove(queue)
            client.connections = max(0, client.connections - 1)
            if client.connections == 0:
                client.left_at = self.clock()
        self._send_presence()

    def rename(self, cid, name):
        name = clean_name(name)
        with self._lock:
            client = self._clients.get(cid)
            if client is None or client.name == name:
                return
            old, client.name = client.name, name
        self.log(name, "renamed (was {})".format(old), ok=True, by=cid)
        self._send_presence()
        self._send_lease()

    def set_page(self, cid, page):
        with self._lock:
            client = self._clients.get(cid)
            if client is None:
                return
            client.page = str(page or "")[:40]
        self._send_presence()

    def name_of(self, cid, ip=None):
        with self._lock:
            client = self._clients.get(cid)
        if client is not None:
            return client.name
        return "someone" + (" at {}".format(ip) if ip else "")

    def clients_on(self, page):
        with self._lock:
            return {c.id for c in self._clients.values() if c.connections and c.page == page}

    def presence(self):
        with self._lock:
            return [c.public() for c in sorted(self._clients.values(), key=lambda c: c.since)]

    def _send_presence(self):
        self.broadcast({"type": "presence", "clients": self.presence()})

    # ---- lease --------------------------------------------------------------------

    def lease(self):
        with self._lock:
            if self._holder is None:
                return {"holder": None, "since": None, "idle_left": None}
            client = self._clients.get(self._holder)
            return {
                "holder": {"id": self._holder, "name": client.name if client else "?"},
                "since": self._since,
                "idle_left": max(0.0, round(self.idle_timeout - (self.clock() - self._last_activity), 1)),
            }

    def _send_lease(self):
        self.broadcast(dict(self.lease(), type="lease"))

    def take(self, cid):
        with self._lock:
            client = self._clients.get(cid)
            if client is None or client.connections == 0:
                raise LockedError("open the page first (no live connection for this tab)")
            if self._holder == cid:
                self._last_activity = self.clock()
                return
            if self._holder is not None:
                raise LockedError("the arm is controlled by {}".format(self.name_of(self._holder)))
            now = self.clock()
            self._holder, self._since, self._last_activity = cid, now, now
            self._ui = None
        self.log(client.name, "took control", ok=True, by=cid)
        self._send_lease()

    def release(self, cid, reason=None):
        with self._lock:
            if self._holder != cid:
                return False
            self._holder = self._since = self._last_activity = None
            self._ui = None
            capped = self._speed > SPEED_LIMIT
            if capped:
                self._speed = SPEED_LIMIT
            self._fast = False
            speed = self._speed_message()
        if capped:
            self.broadcast(speed)
        self.log(self.name_of(cid), "released control" + (" ({})".format(reason) if reason else ""),
                 ok=True, by=None if reason else cid)
        self._send_lease()
        self.broadcast({"type": "ui", "from": None, "state": None})
        return True

    # ---- speed, shared by everybody -----------------------------------------------

    @property
    def speed(self):
        with self._lock:
            return self._speed

    def _speed_message(self):
        return {"type": "speed", "value": self._speed, "fast": self._fast,
                "limit": SPEED_LIMIT, "max": SPEED_MAX}

    def set_speed(self, cid, value, fast=False):
        """Operator only. Above SPEED_LIMIT it needs fast=True (the page's warning, accepted)."""
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= SPEED_MAX:
            raise ValueError("speed must be a whole number from 1 to {}".format(SPEED_MAX))
        if value > SPEED_LIMIT and not fast:
            raise ValueError("speed above {} needs the fast range unlocked".format(SPEED_LIMIT))
        with self._lock:
            if self._holder is None:
                raise LockedError("take control first")
            if self._holder != cid:
                raise LockedError("the arm is controlled by {}".format(self.name_of(self._holder)))
            changed = value != self._speed
            self._speed, self._fast = value, bool(fast) and value > SPEED_LIMIT
            self._last_activity = self.clock()
            message = self._speed_message()
        self.broadcast(message)
        if changed:
            self.log(self.name_of(cid), "speed {}".format(value), ok=True, by=cid)
        return message

    def touch(self, cid):
        with self._lock:
            if self._holder == cid:
                self._last_activity = self.clock()

    def is_operator(self, cid):
        with self._lock:
            return cid is not None and self._holder == cid

    def tick(self):
        """Once a second: idle lease, clients gone for good, lease countdown."""
        now = self.clock()
        gone = []
        with self._lock:
            for cid, client in list(self._clients.items()):
                if client.connections == 0 and client.left_at and now - client.left_at > self.grace:
                    gone.append(client)
            holder = self._holder
            idle = holder is not None and now - self._last_activity > self.idle_timeout
        if idle:
            self.release(holder, "idle {:.0f} s".format(self.idle_timeout))
        for client in gone:
            if self._holder == client.id:
                self.release(client.id, "left the page")
            with self._lock:
                if self._clients.get(client.id) is client and client.connections == 0:
                    del self._clients[client.id]
            self.log(client.name, "left", ok=True, by=client.id)
        if gone:
            self._send_presence()
        if self._holder is not None:
            self._send_lease()   # keeps the idle countdown moving on every page

    # ---- operator's control panel, mirrored to the viewers ------------------------

    def ui(self, cid, state):
        if not self.is_operator(cid):
            return
        self.touch(cid)
        with self._lock:
            self._ui = {"from": cid, "name": self.name_of(cid), "state": state}
            message = dict(self._ui, type="ui")
        self.broadcast(message, exclude=cid)

    # ---- commands -----------------------------------------------------------------

    def run(self, cid, action, fn, operator=True, ip=None):
        """Run fn() for `cid`, logging it for everybody.

        operator=True: only the lease holder, and only one command at a time.
        operator=False (STOP): anyone, never blocked by a running command.
        """
        who = self.name_of(cid, ip)
        if operator:
            with self._lock:
                holder = self._holder
            if holder is None:
                raise LockedError("take control first")
            if holder != cid:
                raise LockedError("the arm is controlled by {}".format(self.name_of(holder)))
            if not self._command_lock.acquire(blocking=False):
                raise BusyError("another command is still running, try again")
        try:
            result = fn()
        except Exception as e:
            self.log(who, action, ok=False, detail=str(e), by=cid)
            raise
        finally:
            if operator:
                self._command_lock.release()
        self.touch(cid)
        self.log(who, action, ok=True, by=cid)
        return result

    # ---- activity log -------------------------------------------------------------

    def log(self, who, action, ok=True, detail=None, by=None):
        """`by` is the client id (None for the arm itself), so a tab can skip its own entries."""
        entry = {"t": self.clock(), "who": who, "action": action, "ok": ok, "detail": detail, "by": by}
        with self._lock:
            self._activity.append(entry)
        self.broadcast({"type": "activity", "entry": entry})
        return entry

    def activity(self):
        with self._lock:
            return list(self._activity)


async def drain(queue, send):
    """Forward one websocket's queue until it is cancelled."""
    while True:
        message = await queue.get()
        await send(message)


def new_queue():
    return asyncio.Queue()
