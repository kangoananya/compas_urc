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
  global bcode = {int_list}
{param_lists}
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
            bcode[w] = read_input_integer_register({i_code} + k)
{param_reads}
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
      i = ridx
      c = bcode[i]
      if c == {CODE_MOVEL}:
        pose = p[0, 0, 0, 0, 0, 0]
        pose[0] = b0[i]
        pose[1] = b1[i]
        pose[2] = b2[i]
        pose[3] = b3[i]
        pose[4] = b4[i]
        pose[5] = b5[i]
        movel(pose, a=b7[i], v=b6[i], r=b8[i])
        expect_blend = b8[i] > 0
      elif c == {CODE_MOVEP}:
        pose = p[0, 0, 0, 0, 0, 0]
        pose[0] = b0[i]
        pose[1] = b1[i]
        pose[2] = b2[i]
        pose[3] = b3[i]
        pose[4] = b4[i]
        pose[5] = b5[i]
        movep(pose, a=b7[i], v=b6[i], r=b8[i])
        expect_blend = b8[i] > 0
      elif c == {CODE_MOVEJ_JOINTS}:
        q = [b0[i], b1[i], b2[i], b3[i], b4[i], b5[i]]
        movej(q, a=b7[i], v=b6[i], r=b8[i])
        expect_blend = b8[i] > 0
      elif c == {CODE_MOVEJ_POSE}:
        pose = p[0, 0, 0, 0, 0, 0]
        pose[0] = b0[i]
        pose[1] = b1[i]
        pose[2] = b2[i]
        pose[3] = b3[i]
        pose[4] = b4[i]
        pose[5] = b5[i]
        movej(pose, a=b7[i], v=b6[i], r=b8[i])
        expect_blend = b8[i] > 0
      elif c == {CODE_SET_DIGITAL}:
        set_standard_digital_out(floor(b0[i]), b1[i] > 0.5)
      elif c == {CODE_SET_TOOL_DIGITAL}:
        set_tool_digital_out(floor(b0[i]), b1[i] > 0.5)
      elif c == {CODE_SET_ANALOG}:
        set_standard_analog_out(floor(b0[i]), b1[i])
      elif c == {CODE_WAIT_TIME}:
        sleep(b0[i])
        expect_blend = False
      elif c == {CODE_SET_TCP}:
        tcp = p[0, 0, 0, 0, 0, 0]
        tcp[0] = b0[i]
        tcp[1] = b1[i]
        tcp[2] = b2[i]
        tcp[3] = b3[i]
        tcp[4] = b4[i]
        tcp[5] = b5[i]
        set_tcp(tcp)
      elif c == {CODE_GET_FRAME}:
        write_result(get_actual_tcp_pose())
        expect_blend = False
      elif c == {CODE_GET_JOINTS}:
        write_result(get_actual_joint_positions())
        expect_blend = False
      elif c == {CODE_READ_DIGITAL}:
        value = 0.0
        if get_standard_digital_in(floor(b0[i])):
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
        while get_standard_digital_in(floor(b0[i])) != (b1[i] > 0.5):
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
    param_lists = '\n'.join('  global b{} = {}'.format(j, _float_list(buffer_size)) for j in range(NUM_PARAMS))
    param_reads = '\n'.join(
        '            b{j}[w] = read_input_float_register({d} + k * {n} + {j})'.format(j=j, d=register_offset, n=NUM_PARAMS)
        for j in range(NUM_PARAMS))
    return TEMPLATE.format(
        buf_size=buffer_size,
        int_list='[' + ', '.join(['0'] * buffer_size) + ']',
        param_lists=param_lists,
        param_reads=param_reads,
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


def _float_list(size):
    return '[' + ', '.join(['0.0'] * size) + ']'
