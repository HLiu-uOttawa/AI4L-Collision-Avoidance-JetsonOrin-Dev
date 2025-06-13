import threading
import time
import random

class StatusCollector:
    def __init__(self):
        self.status = {
            "fps": 0.0,
            "tracks": 0,
            "status": "Initializing"
        }
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._update_loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _update_loop(self):
        while self._running:
            # Simulate data collection; replace this with real logic
            with self._lock:
                self.status["fps"] = random.uniform(15.0, 30.0)
                self.status["tracks"] = random.randint(0, 5)
                self.status["status"] = "Running"
            time.sleep(0.5)

    def get_status(self):
        with self._lock:
            return self.status.copy()


# --- app.py ---
from dash import Dash, dcc, html
import dash_daq as daq
from dash.dependencies import Input, Output

collector = StatusCollector()
collector.start()

app = Dash(__name__)

app.layout = html.Div([
    html.H3("Jetson Real-Time Monitor"),
    daq.LEDDisplay(id='fps-display', label="FPS", value="0.0"),
    daq.LEDDisplay(id='track-display', label="Tracks", value="0"),
    html.Div(id='status-div', children="Status: Initializing"),
    dcc.Interval(id='interval', interval=500, n_intervals=0)
])

@app.callback(
    [Output("fps-display", "value"),
     Output("track-display", "value"),
     Output("status-div", "children")],
    [Input("interval", "n_intervals")]
)
def update_ui(n):
    status = collector.get_status()
    return f"{status['fps']:.1f}", str(status['tracks']), f"Status: {status['status']}"

def launch_dashboard():
    app.run(debug=False, host='0.0.0.0', port=8050)

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

if __name__ == '__main__':
    app.run(debug=False, host='0.0.0.0', port=8050)
