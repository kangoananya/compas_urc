"""Minimal client for the Universal Robots RTDE protocol (version 2).

Only what compas_urc needs: setting up one output and one input recipe,
receiving output packages in a background thread and sending input packages.
Protocol reference: UR support article "Real-Time Data Exchange (RTDE) Guide".
"""
import socket
import struct
import threading

RTDE_PORT = 30004
PROTOCOL_VERSION = 2

# package types
REQUEST_PROTOCOL_VERSION = 86  # 'V'
GET_URCONTROL_VERSION = 118  # 'v'
TEXT_MESSAGE = 77  # 'M'
DATA_PACKAGE = 85  # 'U'
SETUP_OUTPUTS = 79  # 'O'
SETUP_INPUTS = 73  # 'I'
START = 83  # 'S'
PAUSE = 80  # 'P'

HEADER = struct.Struct('>HB')

TYPE_FORMATS = {
    'BOOL': '?',
    'UINT8': 'B',
    'UINT32': 'I',
    'UINT64': 'Q',
    'INT32': 'i',
    'DOUBLE': 'd',
    'VECTOR3D': '3d',
    'VECTOR6D': '6d',
    'VECTOR6INT32': '6i',
    'VECTOR6UINT32': '6I',
}


class RTDEError(Exception):
    pass


class Recipe(object):
    def __init__(self, recipe_id, names, types):
        for name, type_ in zip(names, types):
            if type_ not in TYPE_FORMATS:
                raise RTDEError('RTDE field {} not available: {}'.format(name, type_))
        self.id = recipe_id
        self.names = names
        self.types = types
        self.struct = struct.Struct('>B' + ''.join(TYPE_FORMATS[t] for t in types))

    def unpack(self, payload):
        values = self.struct.unpack(payload)[1:]
        result = {}
        i = 0
        for name, type_ in zip(self.names, self.types):
            fmt = TYPE_FORMATS[type_]
            if fmt[0].isdigit():
                n = int(fmt[0])
                result[name] = list(values[i:i + n])
                i += n
            else:
                result[name] = values[i]
                i += 1
        return result

    def pack(self, values):
        flat = []
        for name, type_ in zip(self.names, self.types):
            value = values[name]
            if TYPE_FORMATS[type_][0].isdigit():
                flat.extend(value)
            else:
                flat.append(value)
        return self.struct.pack(self.id, *flat)


class RTDEConnection(object):
    """Connection to the RTDE interface of a UR controller.

    Call :meth:`setup_outputs` and :meth:`setup_inputs`, then :meth:`start`.
    After starting, every received output package is passed to ``on_data``.
    """

    def __init__(self, ip, port=RTDE_PORT, timeout=5.0):
        self.ip = ip
        self.socket = socket.create_connection((ip, port), timeout=timeout)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._buffer = b''
        self._send_lock = threading.Lock()
        self._thread = None
        self._running = False
        self.output_recipe = None
        self.input_recipe = None
        self.on_data = None
        self.on_disconnect = None
        self.controller_version = None

        if not self._request(REQUEST_PROTOCOL_VERSION, struct.pack('>H', PROTOCOL_VERSION))[0]:
            raise RTDEError('Controller does not support RTDE protocol version {}'.format(PROTOCOL_VERSION))
        payload = self._request(GET_URCONTROL_VERSION, b'')
        self.controller_version = struct.unpack('>IIII', payload)

    # low level

    def _send(self, package_type, payload):
        data = HEADER.pack(HEADER.size + len(payload), package_type) + payload
        with self._send_lock:
            self.socket.sendall(data)

    def _receive(self):
        while True:
            if len(self._buffer) >= HEADER.size:
                size, package_type = HEADER.unpack_from(self._buffer)
                if len(self._buffer) >= size:
                    payload = self._buffer[HEADER.size:size]
                    self._buffer = self._buffer[size:]
                    return package_type, payload
            chunk = self.socket.recv(4096)
            if not chunk:
                raise RTDEError('RTDE connection closed by controller')
            self._buffer += chunk

    def _request(self, package_type, payload):
        # only used before start(), when no receive thread is running
        self._send(package_type, payload)
        while True:
            reply_type, reply = self._receive()
            if reply_type == package_type:
                return reply
            # text messages and stray data packages are skipped during setup

    # setup

    def setup_outputs(self, names, frequency):
        payload = struct.pack('>d', frequency) + ','.join(names).encode('utf-8')
        reply = self._request(SETUP_OUTPUTS, payload)
        types = reply[1:].decode('utf-8').split(',')
        self.output_recipe = Recipe(reply[0], names, types)

    def setup_inputs(self, names):
        reply = self._request(SETUP_INPUTS, ','.join(names).encode('utf-8'))
        types = reply[1:].decode('utf-8').split(',')
        if 'IN_USE' in types:
            used = [n for n, t in zip(names, types) if t == 'IN_USE']
            raise RTDEError('RTDE inputs already used by another client: {}'.format(', '.join(used)))
        self.input_recipe = Recipe(reply[0], names, types)

    def start(self):
        if not self._request(START, b'')[0]:
            raise RTDEError('Controller refused to start RTDE synchronization')
        self.socket.settimeout(None)
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    # runtime

    def send_inputs(self, values):
        self._send(DATA_PACKAGE, self.input_recipe.pack(values))

    def _receive_loop(self):
        try:
            while self._running:
                package_type, payload = self._receive()
                if package_type == DATA_PACKAGE and self.on_data:
                    self.on_data(self.output_recipe.unpack(payload))
        except (OSError, RTDEError) as error:
            if self._running and self.on_disconnect:
                self.on_disconnect(error)

    def close(self):
        if not self._running and self.socket is None:
            return
        self._running = False
        try:
            self._send(PAUSE, b'')
        except OSError:
            pass
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
