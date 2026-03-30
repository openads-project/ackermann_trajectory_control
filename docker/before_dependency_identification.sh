# overwrite released ros2_tracing packages with fork to support
# 'message-link instrumentation' and 'dual-session mode' in jazzy
mkdir -p /tmp/src
cd /tmp
source /opt/ros/jazzy/setup.bash
git clone --branch jazzy-ika https://github.com/RaphvK/ros2_tracing.git src/ros2_tracing
apt update && rosdep update && rosdep install -i --from-paths src -y
colcon build --install-base /opt/ros/jazzy/ --merge-install
rm -r src
