#!/bin/bash

./run_docker.sh

session="robot"

tmux start-server

tmux new-session -d -s $session
tmux split-window -v
tmux selectp -t 0
tmux split-window -v
tmux selectp -t 2
tmux split-window -v
tmux selectp -t 0
tmux split-window -h
tmux selectp -t 2
tmux split-window -h
tmux selectp -t 4
tmux split-window -h
tmux selectp -t 6
tmux split-window -h

tmux setw -g mouse on

tmux_send_all() {
	for _pane in $(tmux list-panes -F '#P'); do
		tmux send-keys -t ${_pane} "$@"
	done
}

tmux_send_all "cd /home/lois" C-m
tmux_send_all "source /opt/ros/galactic/setup.bash" C-m
tmux_send_all "source ./colcon/install/setup.bash" C-m

tmux selectp -t 0
tmux send-keys "cd ./chrony" C-m
tmux send-keys "./chrony.sh" C-m

tmux selectp -t 1
tmux send-keys "ros2 launch launch/ecu.py" C-m

tmux selectp -t 2
tmux send-keys "ros2 launch launch/pls.py" C-m

tmux selectp -t 3
tmux send-keys "cd lois_frontend/webgui" C-m
tmux send-keys "python3 -m http.server 8000" C-m

tmux selectp -t 4
tmux send-keys "ros2 launch launch/realsense.py" C-m

tmux selectp -t 5
tmux send-keys "sleep 10" C-m
tmux send-keys "ros2 launch launch/wideangle_cam.py" C-m

tmux selectp -t 6
tmux send-keys "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

tmux new-window -t $session:1 -n scratch
tmux select-window -t $session:0
tmux attach-session -t $session
