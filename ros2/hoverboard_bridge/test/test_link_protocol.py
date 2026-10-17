from hoverboard_bridge import link_protocol as lp

CMD_GOLDEN = bytes([0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x9C, 0xFF])
STATE_GOLDEN = bytes([0x10, 0x07, 0x00, 0x40, 0xE2, 0x01, 0x00, 0x18, 0xFC, 0xFF, 0xFF,
                      0xD0, 0x07, 0x00, 0x00, 0x32, 0x00, 0xCE, 0xFF, 0x0A, 0x0F, 0x45,
                      0x01, 0x05, 0x64, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00])


def test_cmd_wheels_matches_firmware_bytes():
    assert lp.encode_cmd_wheels(1, 100, -100) == CMD_GOLDEN


def test_wheel_state_decodes_firmware_bytes():
    msg = lp.decode(STATE_GOLDEN)
    assert (msg.type, msg.seq, msg.stamp_ms) == (lp.MSG_WHEEL_STATE, 7, 123456)
    state = lp.parse_wheel_state(msg.body)
    assert state == lp.WheelState(-1000, 2000, 50, -50, 3850, 325, 0x05, 100, 2)


def test_short_inputs_are_rejected():
    assert lp.decode(b'\x10\x00') is None
    assert lp.parse_wheel_state(b'\x00' * 10) is None
    assert lp.parse_imu(b'\x00' * 10) is None
