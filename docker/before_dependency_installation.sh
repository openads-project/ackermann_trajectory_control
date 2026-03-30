# remove once [https://github.com/ros2/ros2_tracing/issues/211] is solved in released version
mkdir -p /tmp/src
cd /tmp
git clone --branch jazzy-ika https://github.com/RaphvK/ros2_tracing.git src/ros2_tracing
rosdep install -i --from-paths src -y
colcon build --install-base /opt/ros/jazzy/ --merge-install
rm -r src
