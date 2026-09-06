"""Run one Gazebo Sim instance and reliably clean up its process group.

The stock ros_gz_sim launcher starts ``gz sim`` through a shell wrapper.  If the
launch process is interrupted at an unlucky time, the Gazebo server can outlive
the wrapper.  Several servers for the same world then publish identically named
entities on the same Gazebo Transport topics, which looks like a flickering or
teleporting vehicle in the GUI.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Iterable


def _cmdline(pid: int) -> list[str]:
    try:
        raw = Path(f'/proc/{pid}/cmdline').read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return []
    return [part.decode(errors='replace') for part in raw.split(b'\0') if part]


def _is_gz_sim(arguments: list[str]) -> bool:
    return any(
        Path(arguments[index]).name == 'gz' and arguments[index + 1] == 'sim'
        for index in range(len(arguments) - 1)
    )


def _argument_resolves_to_world(argument: str, pid: int, world: Path) -> bool:
    if argument.startswith('-'):
        return False
    candidate = Path(argument)
    if not candidate.is_absolute():
        try:
            candidate = Path(f'/proc/{pid}/cwd').resolve() / candidate
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            return False
    try:
        return candidate.resolve() == world
    except (OSError, RuntimeError):
        return False


def find_existing_servers(world: Path, excluded_pids: Iterable[int] = ()) -> list[tuple[int, list[str]]]:
    excluded = set(excluded_pids)
    matches: list[tuple[int, list[str]]] = []
    for proc_entry in Path('/proc').iterdir():
        if not proc_entry.name.isdigit():
            continue
        pid = int(proc_entry.name)
        if pid in excluded:
            continue
        arguments = _cmdline(pid)
        if not _is_gz_sim(arguments):
            continue
        if any(_argument_resolves_to_world(arg, pid, world) for arg in arguments):
            matches.append((pid, arguments))
    return sorted(matches)


def _acquire_world_lock(world: Path):
    digest = hashlib.sha256(str(world).encode()).hexdigest()[:16]
    lock_path = Path('/tmp') / f'xycar_gz_sim_{digest}.lock'
    lock_stream = lock_path.open('a+', encoding='utf-8')
    try:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_stream.close()
        raise RuntimeError(
            f'another xycar Gazebo supervisor already owns {lock_path}') from None
    lock_stream.seek(0)
    lock_stream.truncate()
    lock_stream.write(f'pid={os.getpid()}\nworld={world}\n')
    lock_stream.flush()
    return lock_stream


def _signal_process_group(process: subprocess.Popen, sig: signal.Signals) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def run(args: argparse.Namespace) -> int:
    world = Path(args.world).expanduser().resolve()
    if not world.is_file():
        print(f'[gz_supervisor] world file not found: {world}', file=sys.stderr)
        return 2

    try:
        lock_stream = _acquire_world_lock(world)
    except RuntimeError as error:
        print(f'[gz_supervisor] ERROR: {error}', file=sys.stderr)
        print('[gz_supervisor] Stop the first launch with Ctrl+C before starting another.',
              file=sys.stderr)
        return 3

    existing = find_existing_servers(world, excluded_pids=(os.getpid(), os.getppid()))
    if existing:
        print('[gz_supervisor] ERROR: an existing Gazebo process is already running this world:',
              file=sys.stderr)
        for pid, command in existing:
            print(f"  PID {pid}: {' '.join(command)}", file=sys.stderr)
        print('[gz_supervisor] Stop that process, then launch the simulation again.',
              file=sys.stderr)
        lock_stream.close()
        return 4

    command = ['gz', 'sim', '-r']
    if not args.gui:
        command.extend(['-s', '--headless-rendering'])
    elif args.gui_config:
        command.extend(['--gui-config', str(Path(args.gui_config).expanduser().resolve())])
    command.extend([str(world), '--force-version', str(args.version)])

    environment = os.environ.copy()
    if args.server_config:
        environment['GZ_SIM_SERVER_CONFIG_PATH'] = str(
            Path(args.server_config).expanduser().resolve())

    print(f"[gz_supervisor] starting: {' '.join(command)}", flush=True)
    process = subprocess.Popen(command, env=environment, start_new_session=True)
    print(f'[gz_supervisor] Gazebo process group: {process.pid}', flush=True)

    shutdown_signal: signal.Signals | None = None
    shutdown_started = 0.0

    def request_shutdown(signum, _frame):
        nonlocal shutdown_signal, shutdown_started
        if shutdown_signal is None:
            shutdown_signal = signal.Signals(signum)
            shutdown_started = time.monotonic()
            print(f'[gz_supervisor] forwarding {shutdown_signal.name} to Gazebo', flush=True)
            _signal_process_group(process, shutdown_signal)

    previous_handlers = {
        sig: signal.signal(sig, request_shutdown)
        for sig in (signal.SIGINT, signal.SIGTERM)
    }

    try:
        while process.poll() is None:
            time.sleep(0.1)
            if shutdown_signal is None:
                continue
            elapsed = time.monotonic() - shutdown_started
            # The gz Ruby launcher already performs an orderly GUI/server
            # shutdown and has its own escalation. Sending a second SIGTERM
            # while it reaps a child can trigger a harmless ESRCH traceback.
            # Only use a final safety kill if that cleanup genuinely stalls.
            if elapsed >= 15.0:
                print('[gz_supervisor] Gazebo did not stop; sending SIGKILL', file=sys.stderr,
                      flush=True)
                _signal_process_group(process, signal.SIGKILL)
                break
        return_code = process.wait()
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        if process.poll() is None:
            _signal_process_group(process, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                _signal_process_group(process, signal.SIGKILL)
                process.wait()
        lock_stream.close()

    if shutdown_signal is not None:
        return 0
    return return_code


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Run exactly one Gazebo Sim server for an Xycar world.')
    parser.add_argument('--world', required=True)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--gui-config', default='')
    parser.add_argument('--server-config', default='')
    parser.add_argument('--version', type=int, default=8)
    sys.exit(run(parser.parse_args()))


if __name__ == '__main__':
    main()
