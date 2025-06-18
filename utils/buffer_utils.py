MAX_LEN = {
    "image": int(10 * 1.3),
    "radar": int(25 * 1.3),
    "imu": int(200 * 1.3),
    "gps": int(5 * 1.3)
}

def create_sensor_buffers(manager):
    buffers = {
        "image": {
            "data": manager.list(),
            "maxlength": int(20 * 1.3)
        },
        "radar": {
            "data": manager.list(),
            "maxlength": int(25 * 1.3)
        },
        "imu": {
            "data": manager.list(),
            "maxlength": int(200 * 1.3)
        },
        "gps": {
            "data": manager.list(),
            "maxlength": int(5 * 1.3)
        }
    }

    return buffers