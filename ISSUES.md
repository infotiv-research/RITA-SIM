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

