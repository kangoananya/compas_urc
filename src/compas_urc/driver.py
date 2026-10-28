"""URScript driver that runs on the robot.

The driver is a small, fixed program. Instructions are streamed to it through
RTDE input registers and kept in a ring buffer on the robot, so the length of
a path is not limited by the size of an uploaded script.

Register layout (all relative to the register offset):

    input int     +0          sequence id of the last instruction in the batch
    input int     +1          number of instructions in the batch
    input int     +2          session id, echoed back when the driver is ready
    input int     +3          resume: WaitForResume instructions up to this sequence id may continue
    input int     +4 ...      instruction codes, one per batch slot
    input double  +0 ...      NUM_PARAMS parameters per batch slot

    output int    +0          ack: sequence id of the last batch copied to the buffer
    output int    +1          executed: sequence id of the last executed instruction
    output int    +2          session id echo
    output int    +3          underruns: times the buffer ran empty during a blend
    output int    +4          sequence id of the instruction whose result is in the output doubles
    output int    +5          sequence id of the WaitForResume the robot is waiting at, 0 if none
    output double +0 ... +5   result values

Two loops run on the robot:

* the ``receiver`` thread copies new batches from the registers into the buffer
  and acknowledges them,
* the main loop takes instructions from the buffer and executes them.

Because the main loop only reads from robot memory, the next motion is
available immediately when a blended move returns, and digital outputs are
switched in sequence with the motion.

Instructions are numbered from 1 in the order they are sent, so the sequence
id of the instruction being executed is always ``executed + 1``.
"""

NUM_PARAMS = 9
NUM_RESULTS = 6

# The controller silently ignores a script with a line of about 1000 characters
# or more (seen on PolyScope 5.25), so the buffer lists are split into chunks
# with short literals.
CHUNK_SIZE = 50

# instruction codes, must match the main loop below
CODE_MOVEL = 1
CODE_MOVEJ_JOINTS = 2
CODE_MOVEJ_POSE = 3
CODE_SET_DIGITAL = 4
CODE_SET_TOOL_DIGITAL = 5
CODE_SET_ANALOG = 6
CODE_WAIT_TIME = 7
CODE_SET_TCP = 8
CODE_MOVEP = 9
CODE_GET_FRAME = 10
CODE_GET_JOINTS = 11
CODE_READ_DIGITAL = 12
CODE_WAIT_RESUME = 13
CODE_WAIT_DIGITAL_INPUT = 14
CODE_END = 99

TEMPLATE = """def compas_urc_driver():
  textmsg("compas_urc: driver starting")
  global buf_size = {buf_size}
{buffer_lists}
  global widx = 0
  global ridx = 0
  global count = 0
  global last_seq = 0
  global executed = 0
  global underruns = 0
  global running = True
  global expect_blend = False

  write_output_integer_register({o_ack}, 0)
  write_output_integer_register({o_executed}, 0)
  write_output_integer_register({o_underruns}, 0)
  write_output_integer_register({o_result_seq}, 0)
  write_output_integer_register({o_waiting}, 0)

  # results are written to the output registers, then the driver waits one
  # cycle so the PC receives them before the next result can overwrite them
  def write_result(values):
    write_output_float_register({o_result} + 0, values[0])
    write_output_float_register({o_result} + 1, values[1])
    write_output_float_register({o_result} + 2, values[2])
    write_output_float_register({o_result} + 3, values[3])
    write_output_float_register({o_result} + 4, values[4])
    write_output_float_register({o_result} + 5, values[5])
    write_output_integer_register({o_result_seq}, executed + 1)
    sync()
  end

  thread receiver():
    while True:
      seq = read_input_integer_register({i_seq})
      if seq != last_seq:
        n = read_input_integer_register({i_count})
        if n <= buf_size - count:
          k = 0
          w = widx
          while k < n:
            # receiver variables have their own names (r...) so they never mix with the main loop's
            rc = read_input_integer_register({i_code} + k)
{param_reads}
            rch = floor(w / {chunk_size})
            rj = w - rch * {chunk_size}
{store_block}
            w = w + 1
            if w >= buf_size:
              w = 0
            end
            k = k + 1
          end
          enter_critical
          widx = w
          count = count + n
          exit_critical
          last_seq = seq
          write_output_integer_register({o_ack}, seq)
        end
      end
      sync()
    end
  end

  receiver_thrd = run receiver()
  write_output_integer_register({o_session}, read_input_integer_register({i_session}))
  textmsg("compas_urc: driver ready")

  while running:
    if count > 0:
      ch = floor(ridx / {chunk_size})
      j = ridx - ch * {chunk_size}
{load_block}
      if c == {CODE_MOVEL}:
        pose = p[0, 0, 0, 0, 0, 0]
        pose[0] = v0
        pose[1] = v1
        pose[2] = v2
        pose[3] = v3
        pose[4] = v4
        pose[5] = v5
        movel(pose, a=v7, v=v6, r=v8)
        expect_blend = v8 > 0
      elif c == {CODE_MOVEP}:
        pose = p[0, 0, 0, 0, 0, 0]
        pose[0] = v0
        pose[1] = v1
        pose[2] = v2
        pose[3] = v3
        pose[4] = v4
        pose[5] = v5
        movep(pose, a=v7, v=v6, r=v8)
        expect_blend = v8 > 0
      elif c == {CODE_MOVEJ_JOINTS}:
        q = [v0, v1, v2, v3, v4, v5]
        movej(q, a=v7, v=v6, r=v8)
        expect_blend = v8 > 0
      elif c == {CODE_MOVEJ_POSE}:
        pose = p[0, 0, 0, 0, 0, 0]
        pose[0] = v0
        pose[1] = v1
        pose[2] = v2
        pose[3] = v3
        pose[4] = v4
        pose[5] = v5
        movej(pose, a=v7, v=v6, r=v8)
        expect_blend = v8 > 0
      elif c == {CODE_SET_DIGITAL}:
        set_standard_digital_out(floor(v0), v1 > 0.5)
      elif c == {CODE_SET_TOOL_DIGITAL}:
        set_tool_digital_out(floor(v0), v1 > 0.5)
      elif c == {CODE_SET_ANALOG}:
        set_standard_analog_out(floor(v0), v1)
      elif c == {CODE_WAIT_TIME}:
        sleep(v0)
        expect_blend = False
      elif c == {CODE_SET_TCP}:
        tcp = p[0, 0, 0, 0, 0, 0]
        tcp[0] = v0
        tcp[1] = v1
        tcp[2] = v2
        tcp[3] = v3
        tcp[4] = v4
        tcp[5] = v5
        set_tcp(tcp)
      elif c == {CODE_GET_FRAME}:
        write_result(get_actual_tcp_pose())
        expect_blend = False
      elif c == {CODE_GET_JOINTS}:
        write_result(get_actual_joint_positions())
        expect_blend = False
      elif c == {CODE_READ_DIGITAL}:
        value = 0.0
        if get_standard_digital_in(floor(v0)):
          value = 1.0
        end
        write_result([value, 0.0, 0.0, 0.0, 0.0, 0.0])
        expect_blend = False
      elif c == {CODE_WAIT_RESUME}:
        write_output_integer_register({o_waiting}, executed + 1)
        while read_input_integer_register({i_resume}) < executed + 1:
          sync()
        end
        write_output_integer_register({o_waiting}, 0)
        expect_blend = False
      elif c == {CODE_WAIT_DIGITAL_INPUT}:
        while get_standard_digital_in(floor(v0)) != (v1 > 0.5):
          sync()
        end
        expect_blend = False
      elif c == {CODE_END}:
        running = False
      end
      ridx = ridx + 1
      if ridx >= buf_size:
        ridx = 0
      end
      enter_critical
      count = count - 1
      exit_critical
      executed = executed + 1
      write_output_integer_register({o_executed}, executed)
    else:
      if expect_blend:
        # the previous move was blended but nothing follows: the robot stops there
        underruns = underruns + 1
        write_output_integer_register({o_underruns}, underruns)
        expect_blend = False
      end
      sync()
    end
  end
  kill receiver_thrd
  textmsg("compas_urc: driver finished")
end
compas_urc_driver()
"""


def generate_driver_script(register_offset, batch_size, buffer_size):
    """Returns the URScript driver for the given register layout."""
    sizes = _chunk_sizes(buffer_size)
    names = ['bcode'] + ['b{}'.format(j) for j in range(NUM_PARAMS)]
    values = ['c'] + ['v{}'.format(j) for j in range(NUM_PARAMS)]

    buffer_lists = []
    for name in names:
        zero = '0' if name == 'bcode' else '0.0'
        for ch, size in enumerate(sizes):
            buffer_lists.append('  global {}_{} = [{}]'.format(name, ch, ', '.join([zero] * size)))

    param_reads = '\n'.join(
        '            r{j} = read_input_float_register({d} + k * {n} + {j})'.format(j=j, d=register_offset, n=NUM_PARAMS)
        for j in range(NUM_PARAMS))
    received = ['rc'] + ['r{}'.format(j) for j in range(NUM_PARAMS)]

    return TEMPLATE.format(
        buf_size=buffer_size,
        chunk_size=CHUNK_SIZE,
        buffer_lists='\n'.join(buffer_lists),
        param_reads=param_reads,
        store_block=_chunk_switch(sizes, '            ', 'rch', ['{}_{{ch}}[rj] = {}'.format(n, r) for n, r in zip(names, received)]),
        load_block=_chunk_switch(sizes, '      ', 'ch', ['{} = {}_{{ch}}[j]'.format(v, n) for n, v in zip(names, values)]),
        i_seq=register_offset,
        i_count=register_offset + 1,
        i_session=register_offset + 2,
        i_resume=register_offset + 3,
        i_code=register_offset + 4,
        o_ack=register_offset,
        o_executed=register_offset + 1,
        o_session=register_offset + 2,
        o_underruns=register_offset + 3,
        o_result_seq=register_offset + 4,
        o_waiting=register_offset + 5,
        o_result=register_offset,
        CODE_MOVEL=CODE_MOVEL,
        CODE_MOVEJ_JOINTS=CODE_MOVEJ_JOINTS,
        CODE_MOVEJ_POSE=CODE_MOVEJ_POSE,
        CODE_SET_DIGITAL=CODE_SET_DIGITAL,
        CODE_SET_TOOL_DIGITAL=CODE_SET_TOOL_DIGITAL,
        CODE_SET_ANALOG=CODE_SET_ANALOG,
        CODE_WAIT_TIME=CODE_WAIT_TIME,
        CODE_SET_TCP=CODE_SET_TCP,
        CODE_MOVEP=CODE_MOVEP,
        CODE_GET_FRAME=CODE_GET_FRAME,
        CODE_GET_JOINTS=CODE_GET_JOINTS,
        CODE_READ_DIGITAL=CODE_READ_DIGITAL,
        CODE_WAIT_RESUME=CODE_WAIT_RESUME,
        CODE_WAIT_DIGITAL_INPUT=CODE_WAIT_DIGITAL_INPUT,
        CODE_END=CODE_END,
    )


def _chunk_sizes(buffer_size):
    full, rest = divmod(buffer_size, CHUNK_SIZE)
    return [CHUNK_SIZE] * full + ([rest] if rest else [])


def _chunk_switch(sizes, indent, variable, statements):
    """An if / elif chain over the chunk index in ``variable`` running ``statements`` on that chunk."""
    lines = []
    for ch in range(len(sizes)):
        keyword = 'if' if ch == 0 else 'elif'
        lines.append('{}{} {} == {}:'.format(indent, keyword, variable, ch))
        lines.extend('{}  {}'.format(indent, s.format(ch=ch)) for s in statements)
    lines.append('{}end'.format(indent))
    return '\n'.join(lines)
