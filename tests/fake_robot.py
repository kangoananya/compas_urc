"""A fake UR controller for testing the client without a robot.

It speaks the RTDE protocol on one port and accepts URScript uploads on
another. When the compas_urc driver is uploaded, it runs a Python imitation
of it: a receiver loop copying batches from the input registers into a ring
buffer every 2 ms, and a main loop executing instructions from the buffer.
It does not run URScript, so it tests the client, not the driver script.
"""
import socket
import struct
import threading
import time

from compas_urc import driver

HEADER = struct.Struct('>HB')
FORMATS = {'INT32': 'i', 'UINT32': 'I', 'DOUBLE': 'd', 'UINT64': 'Q', 'VECTOR6D': '6d'}


def _output_type(name):
    if 'int_register' in name:
        return 'INT32'
    if 'double_register' in name:
        return 'DOUBLE'
    if name in ('actual_TCP_pose', 'actual_q'):
        return 'VECTOR6D'
    if name == 'actual_digital_input_bits':
        return 'UINT64'
    return 'UINT32'


def _listen():
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('127.0.0.1', 0))
    s.listen(5)
    return s


class FakeRobot(object):
    def __init__(self, exec_time=0.0005, version=(5, 15)):
        self.exec_time = exec_time
        self.version = version
        self.frequency = None
        self.inputs = {}
        self.outputs = {'runtime_state': 1, 'robot_status_bits': 3, 'actual_TCP_pose': [0.0] * 6,
                        'actual_q': [0.1, -1.5, 1.5, -1.5, -1.5, 0.0], 'actual_digital_input_bits': 0}
        self.executed_log = []
        self.scripts = []  # every uploaded script
        self.max_count = 0
        self.driver_running = False
        self.lock = threading.Lock()
        self._rtde_server = _listen()
        self._script_server = _listen()
        self.rtde_port = self._rtde_server.getsockname()[1]
        self.script_port = self._script_server.getsockname()[1]
        threading.Thread(target=self._serve_rtde, daemon=True).start()
        threading.Thread(target=self._serve_script, daemon=True).start()

    def _send(self, conn, package_type, payload):
        conn.sendall(HEADER.pack(HEADER.size + len(payload), package_type) + payload)

    def _serve_rtde(self):
        conn, _ = self._rtde_server.accept()
        buffer = b''
        while True:
            while True:
                if len(buffer) >= HEADER.size:
                    size, package_type = HEADER.unpack_from(buffer)
                    if len(buffer) >= size:
                        payload = buffer[HEADER.size:size]
                        buffer = buffer[size:]
                        break
                chunk = conn.recv(65536)
                if not chunk:
                    return
                buffer += chunk
            if package_type == 86:
                self._send(conn, 86, b'\x01')
            elif package_type == 118:
                self._send(conn, 118, struct.pack('>IIII', self.version[0], self.version[1], 0, 0))
            elif package_type == 79:
                self.frequency = struct.unpack('>d', payload[:8])[0]
                self.out_names = payload[8:].decode().split(',')
                self.out_types = [_output_type(n) for n in self.out_names]
                self._send(conn, 79, b'\x01' + ','.join(self.out_types).encode())
                # a text message in between, the client must skip it
                self._send(conn, 77, b'\x02hi\x03src\x01')
            elif package_type == 73:
                self.in_names = payload.decode().split(',')
                self.in_types = ['INT32' if 'int' in n else 'DOUBLE' for n in self.in_names]
                self._send(conn, 73, b'\x02' + ','.join(self.in_types).encode())
            elif package_type == 83:
                self._send(conn, 83, b'\x01')
                threading.Thread(target=self._publish, args=(conn,), daemon=True).start()
            elif package_type == 85:
                values = struct.unpack('>B' + ''.join(FORMATS[t] for t in self.in_types), payload)[1:]
                with self.lock:
                    self.inputs = dict(zip(self.in_names, values))
            elif package_type == 80:
                return

    def _publish(self, conn):
        fmt = '>B' + ''.join(FORMATS[t] for t in self.out_types)
        try:
            while True:
                values = []
                with self.lock:
                    for n in self.out_names:
                        value = self.outputs.get(n, 0)
                        values.extend(value if isinstance(value, list) else [value])
                self._send(conn, 85, struct.pack(fmt, 1, *values))
                time.sleep(0.002)
        except OSError:
            pass

    def _serve_script(self):
        while True:
            conn, _ = self._script_server.accept()
            data = b''
            conn.settimeout(0.5)
            try:
                while True:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    data += chunk
            except socket.timeout:
                pass
            script = data.decode()
            self.scripts.append(script)
            if 'def compas_urc_driver' in script:
                threading.Thread(target=self._run_driver, args=(script,), daemon=True).start()
            elif 'compas_urc_abort' in script:
                self.driver_running = False

    def _run_driver(self, script):
        offset = int(script.split('read_input_integer_register(')[1].split(')')[0])
        buffer_size = int(script.split('global buf_size = ')[1].split('\n')[0])

        def out_int(i):
            return 'output_int_register_{}'.format(offset + i)

        def in_int(i):
            return 'input_int_register_{}'.format(offset + i)

        def in_double(i):
            return 'input_double_register_{}'.format(offset + i)

        ring = [None] * buffer_size
        state = {'widx': 0, 'ridx': 0, 'count': 0, 'last_seq': 0}
        ring_lock = threading.Lock()
        with self.lock:
            self.outputs.update({out_int(0): 0, out_int(1): 0, out_int(3): 0, out_int(4): 0, out_int(5): 0,
                                 'runtime_state': 2})
        self.driver_running = True

        def receiver():
            while self.driver_running:
                with self.lock:
                    inputs = dict(self.inputs)
                seq = inputs.get(in_int(0), 0)
                if seq != state['last_seq']:
                    n = inputs[in_int(1)]
                    if n <= buffer_size - state['count']:
                        w = state['widx']
                        for k in range(n):
                            params = [inputs[in_double(k * driver.NUM_PARAMS + j)] for j in range(driver.NUM_PARAMS)]
                            ring[w] = (inputs[in_int(4 + k)], params)
                            w = (w + 1) % buffer_size
                        with ring_lock:
                            state['widx'] = w
                            state['count'] += n
                            self.max_count = max(self.max_count, state['count'])
                        state['last_seq'] = seq
                        with self.lock:
                            self.outputs[out_int(0)] = seq
                time.sleep(0.002)

        def write_result(values, seq):
            with self.lock:
                for i, value in enumerate(values):
                    self.outputs['output_double_register_{}'.format(offset + i)] = float(value)
                self.outputs[out_int(4)] = seq
            time.sleep(0.004)  # sync(): hold the result at least one cycle

        threading.Thread(target=receiver, daemon=True).start()
        with self.lock:
            self.outputs[out_int(2)] = self.inputs.get(in_int(2), 0)

        executed = 0
        running = True
        while running and self.driver_running:
            if state['count'] > 0:
                code, params = ring[state['ridx']]
                self.executed_log.append((code, params))
                if code in (driver.CODE_MOVEL, driver.CODE_MOVEP, driver.CODE_MOVEJ_POSE):
                    time.sleep(self.exec_time)
                    with self.lock:
                        self.outputs['actual_TCP_pose'] = list(params[:6])
                elif code == driver.CODE_MOVEJ_JOINTS:
                    time.sleep(self.exec_time)
                    with self.lock:
                        self.outputs['actual_q'] = list(params[:6])
                elif code == driver.CODE_GET_FRAME:
                    write_result(self.outputs['actual_TCP_pose'], executed + 1)
                elif code == driver.CODE_GET_JOINTS:
                    write_result(self.outputs['actual_q'], executed + 1)
                elif code == driver.CODE_READ_DIGITAL:
                    bit = self.outputs['actual_digital_input_bits'] >> int(params[0]) & 1
                    write_result([float(bit), 0, 0, 0, 0, 0], executed + 1)
                elif code == driver.CODE_WAIT_RESUME:
                    with self.lock:
                        self.outputs[out_int(5)] = executed + 1
                    while self.driver_running:
                        with self.lock:
                            if self.inputs.get(in_int(3), 0) >= executed + 1:
                                break
                        time.sleep(0.002)
                    with self.lock:
                        self.outputs[out_int(5)] = 0
                elif code == driver.CODE_WAIT_DIGITAL_INPUT:
                    while self.driver_running:
                        bit = self.outputs['actual_digital_input_bits'] >> int(params[0]) & 1
                        if bit == (params[1] > 0.5):
                            break
                        time.sleep(0.002)
                elif code == driver.CODE_END:
                    running = False
                state['ridx'] = (state['ridx'] + 1) % buffer_size
                with ring_lock:
                    state['count'] -= 1
                executed += 1
                with self.lock:
                    self.outputs[out_int(1)] = executed
            else:
                time.sleep(0.002)
        self.driver_running = False
        with self.lock:
            self.outputs['runtime_state'] = 1
