#!/usr/bin/env python3
"""Record planner RAM/RSS and GPU memory during a scenario run."""

from dataclasses import dataclass
from pathlib import Path
import subprocess
import threading


# These strings identify the planner processes that belong to each backend.
PLANNER_PROCESS_PATTERNS = {
    "curobo": ("curobo_trajectory_planner",),
    "cumotion": (
        "move_group",
        "cumotion_planner_node",
        "cumotion_planner_upstream_framefix.py",
    ),
    "hybrid": (
        "move_group",
        "curobo_trajectory_planner",
        "curobo_hybrid_planning_container",
        "component_container_mt",
        "hybrid_move_action_bridge.py",
    ),
    "ompl": ("move_group",),
}


@dataclass
class PlannerProcess:
    pid: int
    ram_mib: float
    command: str


class PlannerResourceRecorder:
    def __init__(
        self,
        planner: str,
        docker_compose: list[str],
        root: Path,
        sample_interval_s: float = 1.0,
    ):
        self.planner = planner
        self.docker_compose = docker_compose
        self.root = root
        self.sample_interval_s = sample_interval_s
        self.stop_event = threading.Event()
        self.ram_memory_samples_mib = []
        self.gpu_memory_samples_mib = []
        self.thread = threading.Thread(target=self.record_until_stopped)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> dict:
        self.stop_event.set()
        self.thread.join()
        return {
            "planner_ram_memory_mib": summarize(self.ram_memory_samples_mib),
            "planner_gpu_memory_mib": summarize(self.gpu_memory_samples_mib),
        }

    def record_until_stopped(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.record_one_sample()
            except Exception:
                pass
            self.stop_event.wait(self.sample_interval_s)

    def record_one_sample(self) -> None:
        planner_processes = self.find_planner_processes()
        ram_memory_mib = self.get_planner_ram_memory_mib(planner_processes)
        gpu_memory_mib = self.get_planner_gpu_memory_mib(planner_processes)

        self.ram_memory_samples_mib.append(ram_memory_mib)
        self.gpu_memory_samples_mib.append(gpu_memory_mib)

    def find_planner_processes(self) -> list[PlannerProcess]:
        container_id = self.get_cumotion_container_id()
        if not container_id:
            return []
        docker_top_output = self.get_container_process_table(container_id)
        planner_patterns = PLANNER_PROCESS_PATTERNS[self.planner]

        planner_processes = []
        for process_line in docker_top_output.splitlines()[1:]:
            try:
                process = self.parse_process_line(process_line)
            except ValueError:
                continue
            if process_matches_any_pattern(process, planner_patterns):
                planner_processes.append(process)

        return planner_processes

    def get_cumotion_container_id(self) -> str:
        try:
            return subprocess.check_output(
                [*self.docker_compose, "ps", "-q", "cumotion"],
                cwd=self.root,
                text=True,
            ).strip()
        except (OSError, subprocess.SubprocessError):
            return ""

    def get_container_process_table(self, container_id: str) -> str:
        try:
            return subprocess.check_output(
                ["docker", "top", container_id, "-eo", "pid,rss,args"],
                text=True,
            )
        except (OSError, subprocess.SubprocessError):
            return ""

    def parse_process_line(self, process_line: str) -> PlannerProcess:
        pid_text, ram_kib_text, command = process_line.split(None, 2)
        return PlannerProcess(
            pid=int(pid_text),
            ram_mib=float(ram_kib_text) / 1024.0,
            command=command,
        )

    def get_planner_ram_memory_mib(self, planner_processes: list[PlannerProcess]) -> float:
        total_ram_memory_mib = 0.0
        for process in planner_processes:
            total_ram_memory_mib += process.ram_mib
        return total_ram_memory_mib

    def get_planner_gpu_memory_mib(self, planner_processes: list[PlannerProcess]) -> float:
        gpu_memory_by_pid = self.get_gpu_memory_by_pid_mib()
        total_gpu_memory_mib = 0.0

        for process in planner_processes:
            total_gpu_memory_mib += gpu_memory_by_pid.get(process.pid, 0.0)

        return total_gpu_memory_mib

    def get_gpu_memory_by_pid_mib(self) -> dict[int, float]:
        try:
            nvidia_smi_output = subprocess.check_output(
                [
                    "nvidia-smi",
                    "--query-compute-apps=pid,used_memory",
                    "--format=csv,noheader,nounits",
                ],
                text=True,
            )
        except (OSError, subprocess.SubprocessError):
            return {}

        gpu_memory_by_pid = {}
        for memory_line in nvidia_smi_output.splitlines():
            try:
                pid_text, gpu_memory_mib_text = memory_line.split(",", 1)
                pid = int(pid_text)
                gpu_memory_mib = float(gpu_memory_mib_text.strip())
            except ValueError:
                continue
            gpu_memory_by_pid[pid] = gpu_memory_by_pid.get(pid, 0.0) + gpu_memory_mib

        return gpu_memory_by_pid


def process_matches_any_pattern(
    process: PlannerProcess, planner_patterns: tuple[str, ...]
) -> bool:
    for pattern in planner_patterns:
        if pattern in process.command:
            return True
    return False


def summarize(samples: list[float]) -> dict[str, float | None]:
    if not samples:
        return {"min": None, "avg": None, "max": None}
    return {
        "min": min(samples),
        "avg": sum(samples) / len(samples),
        "max": max(samples),
    }


def start_planner_resource_recorder(
    planner: str, docker_compose: list[str], root: Path
) -> PlannerResourceRecorder:
    recorder = PlannerResourceRecorder(planner, docker_compose, root)
    recorder.start()
    return recorder


def stop_planner_resource_recorder(recorder: PlannerResourceRecorder) -> dict:
    return recorder.stop()
