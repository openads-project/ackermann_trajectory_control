# overwrite released ros2_tracing packages with fork to support
# 'message-link instrumentation' and 'dual-session mode' in jazzy
mkdir -p /tmp/src
cd /tmp
git clone --branch jazzy-ika https://github.com/RaphvK/ros2_tracing.git src/ros2_tracing
rosdep update && rosdep install -i --from-paths src -y
colcon build --install-base /opt/ros/jazzy/ --merge-install --packages-up-to tracetools tracetools_launch
rm -r src
