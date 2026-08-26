# Lab 5 (Optional) - Scan Matching

Note on why this doesn't match the official handout: that version wants C++, an older ROS1
simulator (racecar_simulator), and a team. None of that fits what I've been doing for the
rest of this project (solo, Python, ROS2), so I rebuilt the same idea from scratch instead -
localizing the car using ICP scan matching, done my own way in this project's simulator.

Everything about how it works, the math questions, and how well it actually performed is in
[WRITEUP.md](WRITEUP.md). The video (`demo.mov`) shows it running against the sim.
