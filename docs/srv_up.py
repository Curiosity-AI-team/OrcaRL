import robot_upstart
j = robot_upstart.Job(name="ros-manual-control")
j.symlink = True
j.add(package="control", filename="launch/manual_control.launch")
j.install()