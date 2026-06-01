import lcm
from payloadMessages import payload_cont_to_cam_msg_t, cam_msg_t

# Subscribe to and wait for msg from payload computer
def wait_for_payload_comp_msg():
    received = {"msg": None}

    def cam_handler(channel, data):
        received["msg"] = payload_cont_to_cam_msg_t.decode(data)
        print("Received message on channel \"%s\"" % channel)
        print("   cont_state = %s" % str(received["msg"].cont_state))
        print("   debug_mode = %s" % str(received["msg"].debug_mode))
        print("")

    lc = lcm.LCM()
    sub = lc.subscribe("PAYLOAD_CAM", cam_handler)

    while received["msg"] is None:
        lc.handle()

    lc.unsubscribe(sub)

    return received["msg"]


# Create a message and fill in data fields
def publish_cam_msg(cam_status=False):
    msg = cam_msg_t()
    msg.cam_status = cam_status

    lc = lcm.LCM()
    lc.publish("CAM_PAYLOAD", msg.encode())

    print("Message published")

