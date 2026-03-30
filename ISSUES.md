# Issues


### xhost issue

Getting these errors:
```
2026-02-26 08:04:48 [23,151ms] [Error] [omni.kit.window.drop_support.drop_support] Cannot setup ExternalDragDrop without a default window
2026-02-26 08:04:48 [23,154ms] [Error] [omni.kit.window.drop_support.drop_support] Cannot setup ExternalDragDrop without a default window
2026-02-26 08:04:48 [23,161ms] [Error] [omni.kit.window.title.title] Cannot set window title without a default window
2026-02-26 08:04:48 [23,161ms] [Error] [omni.kit.window.title.title] Cannot set window title without a default window
2026-02-26 08:04:48 [23,218ms] [Error] [omni.kit.window.title.title] Cannot set window title without a default window
```

solution: `xhost + `


### Issac sim issue

If Isaac Sim does not start:

- Verify NVIDIA drivers on host.
- Verify Docker GPU access:

```bash
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

### Multi display issue

If you get the error below, one simple solution is to disable your other display.
```
Traceback (most recent call last):
  File "/opt/ros/humble/bin/ros2", line 33, in <module>
    sys.exit(load_entry_point('ros2cli==0.18.17', 'console_scripts', 'ros2')())
  File "/opt/ros/humble/bin/ros2", line 25, in importlib_load_entry_point
    return next(matches).load()
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
    module = import_module(match.group('module'))
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1006, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 688, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 883, in exec_module
  File "<frozen importlib._bootstrap>", line 241, in _call_with_frames_removed
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2cli/cli.py", line 22, in <module>
    from rclpy.executors import ExternalShutdownException
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 49, in <module>
    from rclpy.signals import install_signal_handlers
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/signals.py", line 15, in <module>
    from rclpy.exceptions import InvalidHandle
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/exceptions.py", line 15, in <module>
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/impl/implementation_singleton.py", line 32, in <module>
    rclpy_implementation = import_c_library('._rclpy_pybind11', package)
  File "/opt/ros/humble/lib/python3.10/site-packages/rpyutils/import_c_library.py", line 39, in import_c_library
    return importlib.import_module(name, package=package)
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
ImportError: /opt/ros/humble/lib/librcl_logging_spdlog.so: undefined symbol: _ZN6spdlog7details7log_msgC1ENS_10source_locEN3fmt2v817basic_string_viewIcEENS_5level10level_enumES6_
The C extension '/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so' failed to be imported while being present on the system. Please refer to 'https://docs.ros.org/en/{distro}/Guides/Installation-Troubleshooting.html#import-failing-even-with-library-present-on-the-system' for possible solutions
robot_state_publisher not found
Opening scene via startup hook: /ros2_ws/assets/ur10e_robotiq2f-140/main_scene.usd
Loading user config located at: '/root/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/user.config.json'
[Info] [carb] Logging to file: /root/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/4.5/kit_20260305_163221.log
2026-03-05 16:32:21 [0ms] [Warning] [carb.crashreporter-breakpad.plugin] [previous crash] preventing upload of minidump due to user opt-out: '/root/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/26781ca4-7aa5-4dde-3ac75f94-451e54fe.dmp.zip'
2026-03-05 16:32:21 [1ms] [Warning] [carb.crashreporter-breakpad.plugin] [previous crash] preventing upload of minidump due to user opt-out: '/root/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/516bde66-cb08-4124-8d929aab-bc1ffa8d.dmp.zip'
...
...
...
2026-03-05 16:32:31 [2,150ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  telemetrySessionId = '9970911643429079147'
2026-03-05 16:32:31 [2,175ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  terminatedByAbort = '0'
2026-03-05 16:32:31 [2,206ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  totalRamBareMetalMB = '32009'
2026-03-05 16:32:31 [2,230ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  totalRamLimitedMB = '32009'
2026-03-05 16:32:31 [2,256ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  totalSwapBareMetalMB = '40959'
2026-03-05 16:32:31 [2,280ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  totalSwapLimitedMB = '40959'
2026-03-05 16:32:31 [2,305ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  userId = 'default'
2026-03-05 16:32:32 [2,334ms] [Warning] [carb.crashreporter-breakpad.plugin] [crash]  workingDirectory = '/ros2_ws'
2026-03-05 16:32:32 [2,359ms] [Fatal] [carb.crashreporter-breakpad.plugin] [crash] Thread 1219 backtrace follows:
2026-03-05 16:32:32 [2,388ms] [Fatal] [carb.crashreporter-breakpad.plugin] 000: libc.so.6!__sigaction+0x50 (??:?)
2026-03-05 16:32:32 [2,414ms] [Fatal] [carb.crashreporter-breakpad.plugin] 001: libomni.kit.imgui_renderer.plugin.so!void std::vector<void*, std::allocator<void*> >::_M_realloc_insert<void* const&>(__gnu_cxx::__normal_iterator<void**, std::vector<void*, std::allocator<void*> > >, void* const&)+0x35a (??:?)
2026-03-05 16:32:32 [2,440ms] [Fatal] [carb.crashreporter-breakpad.plugin] 002: libomni.kit.imgui_renderer.plugin.so!void std::vector<void*, std::allocator<void*> >::_M_realloc_insert<void* const&>(__gnu_cxx::__normal_iterator<void**, std::vector<void*, std::allocator<void*> > >, void* const&)+0x443 (??:?)
2026-03-05 16:32:32 [2,467ms] [Fatal] [carb.crashreporter-breakpad.plugin] 003: libomni.kit.imgui_renderer.plugin.so!char* std::string::_S_construct<char*>(char*, char*, std::allocator<char> const&, std::forward_iterator_tag)+0x3f13 (??:?)
2026-03-05 16:32:32 [2,492ms] [Fatal] [carb.crashreporter-breakpad.plugin] 004: libomni.kit.imgui_renderer.plugin.so!char* std::string::_S_construct<char*>(char*, char*, std::allocator<char> const&, std::forward_iterator_tag)+0x55d5 (??:?)
2026-03-05 16:32:32 [2,516ms] [Fatal] [carb.crashreporter-breakpad.plugin] 005: libomni.kit.imgui_renderer.plugin.so!carbOnPluginPreStartup+0x175d (??:?)
2026-03-05 16:32:32 [2,541ms] [Fatal] [carb.crashreporter-breakpad.plugin] 006: libomni.kit.imgui_renderer.plugin.so!carbOnPluginPreStartup+0x1a46 (??:?)
2026-03-05 16:32:32 [2,567ms] [Fatal] [carb.crashreporter-breakpad.plugin] 007: libomni.kit.renderer.init.plugin.so!std::_Rb_tree<std::string, std::string, std::_Identity<std::string>, std::less<std::string>, std::allocator<std::string> >::_M_get_insert_hint_unique_pos(std::_Rb_tree_const_iterator<std::string>, std::string const&)+0x461f (??:?)
2026-03-05 16:32:32 [2,591ms] [Fatal] [carb.crashreporter-breakpad.plugin] 008: libomni.kit.renderer.plugin.so!carbOnPluginPreStartup+0x3f5d (??:?)
2026-03-05 16:32:32 [2,617ms] [Fatal] [carb.crashreporter-breakpad.plugin] 009: libomni.kit.imgui_renderer.plugin.so!carbOnPluginPreStartup+0x20eb (??:?)
2026-03-05 16:32:32 [2,643ms] [Fatal] [carb.crashreporter-breakpad.plugin] 010: libomni.kit.imgui_renderer.ext.plugin.so!std::string::find(char, unsigned long) const+0x1a30 (??:?)
2026-03-05 16:32:32 [2,672ms] [Fatal] [carb.crashreporter-breakpad.plugin] 011: libomni.ext.plugin.so!std::string& std::vector<std::string, std::allocator<std::string> >::emplace_back<char const*&>(char const*&)+0x4c59 (??:?)
2026-03-05 16:32:32 [2,695ms] [Fatal] [carb.crashreporter-breakpad.plugin] 012: libomni.ext.plugin.so!std::basic_string<char, std::char_traits<char>, std::allocator<char> >::basic_string(std::string const&, unsigned long, unsigned long)+0x537f (??:?)
2026-03-05 16:32:32 [2,722ms] [Fatal] [carb.crashreporter-breakpad.plugin] 013: libomni.ext.plugin.so!std::string __gnu_cxx::__to_xstring<std::string, char>(int (*)(char*, unsigned long, char const*, __va_list_tag*), unsigned long, char const*, ...)+0x10740 (??:?)
2026-03-05 16:32:32 [2,747ms] [Fatal] [carb.crashreporter-breakpad.plugin] 014: libomni.ext.plugin.so!carbOnPluginPreStartup+0x593 (??:?)
2026-03-05 16:32:32 [2,773ms] [Fatal] [carb.crashreporter-breakpad.plugin] 015: libomni.ext.plugin.so!carbOnPluginPreStartup+0xd33c (??:?)
2026-03-05 16:32:32 [2,797ms] [Fatal] [carb.crashreporter-breakpad.plugin] 016: libomni.ext.plugin.so!carbOnPluginPreStartup+0xe3bb (??:?)
2026-03-05 16:32:32 [2,829ms] [Fatal] [carb.crashreporter-breakpad.plugin] 017: libomni.kit.app.plugin.so!std::string::replace(unsigned long, unsigned long, char const*, unsigned long)+0xaad3 (??:?)
2026-03-05 16:32:32 [2,853ms] [Fatal] [carb.crashreporter-breakpad.plugin] 018: libomni.kit.app.plugin.so!std::string::replace(unsigned long, unsigned long, char const*, unsigned long)+0xb52a (??:?)
2026-03-05 16:32:32 [2,879ms] [Fatal] [carb.crashreporter-breakpad.plugin] 019: libomni.kit.app.plugin.so!_init+0x899a (??:0)
2026-03-05 16:32:32 [2,901ms] [Fatal] [carb.crashreporter-breakpad.plugin] 020: kit!_init+0x90b (??:0)
2026-03-05 16:32:32 [2,929ms] [Fatal] [carb.crashreporter-breakpad.plugin] 021: libc.so.6!__libc_init_first+0x90 (??:?)
2026-03-05 16:32:32 [2,981ms] [Fatal] [carb.crashreporter-breakpad.plugin] 022: libc.so.6!__libc_start_main+0x80 (??:?)
2026-03-05 16:32:32 [3,008ms] [Fatal] [carb.crashreporter-breakpad.plugin] 023: kit!_start+0x29 (??:?)
./control.sh: line 187:  1219 Segmentation fault      (core dumped) ./startup_scripts/post_install_ros2_isaac_start.sh
```
### Out of vram

When running `./control.sh curobo` you might get the following error. Try to change it to `./control.sh cumotion`

```
[ERROR] [curobo_preview_joint_states.py-3]: process has died [pid 21330, exit code 1, cmd '/ros2_ws/install/ur_robotiq_moveit_config/lib/ur_robotiq_moveit_config/curobo_preview_joint_states.py --ros-args --params-file /tmp/launch_params_pwboze99'].
[ERROR] [curobo_human_skeleton_collision_publisher.py-10]: process has died [pid 21344, exit code 1, cmd '/ros2_ws/install/ur_robotiq_moveit_config/lib/ur_robotiq_moveit_config/curobo_human_skeleton_collision_publisher.py --ros-args -r __node:=curobo_human_skeleton_collision_publisher --params-file /tmp/launch_params_0fft98r8'].
[ERROR] [rviz2-9]: process has died [pid 21342, exit code -11, cmd '/opt/ros/humble/lib/rviz2/rviz2 -d /ros2_ws/install/ur_robotiq_curobo_config/share/ur_robotiq_curobo_config/rviz/curobo_minimal.rviz --ros-args --params-file /tmp/launch_params_u6j0nb4v'].
[ERROR] [curobo_world_bridge.py-8]: process has died [pid 21340, exit code 1, cmd '/ros2_ws/install/ur_robotiq_moveit_config/lib/ur_robotiq_moveit_config/curobo_world_bridge.py --ros-args --params-file /tmp/launch_params_qxpa_a1j'].
[ERROR] [curobo_live_collision_spheres.py-7]: process has died [pid 21338, exit code -2, cmd '/ros2_ws/install/ur_robotiq_moveit_config/lib/ur_robotiq_moveit_config/curobo_live_collision_spheres.py --ros-args --params-file /tmp/launch_params_slpn0k2l'].
```


### curobo_human_skeleton_collision_publisher.py not executed

`chmod +x ./install/ur_robotiq_moveit_config/lib/ur_robotiq_moveit_config/curobo_human_skeleton_collision_publisher.py`



### [cumotion_planner_upstream_framefix.py-6] torch.OutOfMemoryError: CUDA out of memory.

It is recomended by Isaac sim to have a GPU with 16gb vram. If you want to run it on a 8gb, it may work with the following adjustment:

```
      cumotion_num_graph_seeds_value=5
      cumotion_num_trajopt_seeds_value=5
```
If you have two GPUs (2*8 gm), both isaac-sim and cumotion use the same GPU and you get this error message as well. You can fix it by specifying which GPU to be used as below:
```
CUDA_VISIBLE_DEVICES=1 ./control.sh cumotion
```

### [Error] [omni.kit.app._impl] [py stderr]

Can be ignored
