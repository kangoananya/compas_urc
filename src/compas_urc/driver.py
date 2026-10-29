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

import os

NUM_PARAMS = 10
NUM_RESULTS = 6

# instruction codes, must match the main loop in driver.script
CODE_MOVEL = 1
CODE_MOVEJ_JOINTS = 2
CODE_MOVEJ_POSE = 3
CODE_SET_DIGITAL = 4
CODE_SET_TOOL_DIGITAL = 5
CODE_SET_ANALOG = 6
CODE_WAIT_TIME = 7
CODE_SET_TOOL = 8
CODE_MOVEP = 9
CODE_GET_FRAME = 10
CODE_GET_JOINTS = 11
CODE_READ_DIGITAL = 12
CODE_WAIT_RESUME = 13
CODE_WAIT_DIGITAL_INPUT = 14
CODE_PRINT_TEXT = 15  # nothing to do on the robot: the client sends the text when it is executed
CODE_END = 99

with open(os.path.join(os.path.dirname(__file__), "driver.script")) as _f:
    TEMPLATE = _f.read()


def generate_driver_script(register_offset, batch_size, buffer_size):
    """Returns the URScript driver for the given register layout."""
    param_lists = "\n".join(
        "  global b{} = {}".format(j, _float_list(buffer_size))
        for j in range(NUM_PARAMS)
    )
    param_reads = "\n".join(
        "            b{j}[w] = read_input_float_register({d} + k * {n} + {j})".format(
            j=j, d=register_offset, n=NUM_PARAMS
        )
        for j in range(NUM_PARAMS)
    )
    return TEMPLATE.format(
        buf_size=buffer_size,
        int_list="[" + ", ".join(["0"] * buffer_size) + "]",
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
        CODE_SET_TOOL=CODE_SET_TOOL,
        CODE_MOVEP=CODE_MOVEP,
        CODE_GET_FRAME=CODE_GET_FRAME,
        CODE_GET_JOINTS=CODE_GET_JOINTS,
        CODE_READ_DIGITAL=CODE_READ_DIGITAL,
        CODE_WAIT_RESUME=CODE_WAIT_RESUME,
        CODE_WAIT_DIGITAL_INPUT=CODE_WAIT_DIGITAL_INPUT,
        CODE_END=CODE_END,
    )


def _float_list(size):
    return "[" + ", ".join(["0.0"] * size) + "]"
