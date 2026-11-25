#!/bin/sh
# Start the simulator: viewer on http://127.0.0.1:8700/ , API under /api
cd "$(dirname "$0")" && exec python3 -m twoarm.server "$@"
