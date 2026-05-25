#!/bin/bash

# Trap SIGINT to kill background processes when the script is terminated
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT


# Determine the root directory of the workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Start Backend
echo "Starting Go Backend Server..."
cd "$WORKSPACE_DIR/workspaces/mission_planner_ui/backend_go" || { echo "Go Backend directory not found"; exit 1; }

echo "Compiling Go binary..."
go build -o main main.go

./main &
BACKEND_PID=$!

# Wait for backend to initialize
sleep 2

# Start Frontend
echo "Starting Frontend..."
cd "$WORKSPACE_DIR/workspaces/mission_planner_ui/frontend" || { echo "Frontend directory not found"; kill $BACKEND_PID; exit 1; }

if [ ! -d "node_modules" ]; then
    echo "Installing frontend dependencies..."
    npm install
fi

npm run dev -- --host &
FRONTEND_PID=$!

# Wait for all processes
echo "Mission Planner UI (Web Side) is running."
echo "Press Ctrl+C to stop all services."
wait $BACKEND_PID $FRONTEND_PID
