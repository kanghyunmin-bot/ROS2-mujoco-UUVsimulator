import argparse
import signal
import time


_STOP_REQUESTED = False


def _request_stop(signum, frame) -> None:
    del signum, frame
    global _STOP_REQUESTED
    _STOP_REQUESTED = True


def _device_candidates(can_interface: str) -> list[str]:
    if can_interface.startswith("socketcan:"):
        return [can_interface]
    return [can_interface, f"socketcan:{can_interface}"]


def _load_dronecan():
    import dronecan
    from dronecan.app.dynamic_node_id import CentralizedServer
    from dronecan.app.node_monitor import NodeMonitor

    return dronecan, NodeMonitor, CentralizedServer


def _open_node(dronecan, can_interface: str, node_id: int):
    last_error = None
    for device in _device_candidates(can_interface):
        try:
            return dronecan.make_node(device, node_id=node_id), device
        except Exception as exc:
            last_error = exc
    raise RuntimeError(f"failed to open {can_interface}: {last_error!r}")


def _is_transfer_error(dronecan, exc: Exception) -> bool:
    transport = getattr(dronecan, "transport", None)
    transfer_error = getattr(transport, "TransferError", None)
    return transfer_error is not None and isinstance(exc, transfer_error)


def _close_allocator(allocator) -> None:
    if allocator is None:
        return
    try:
        allocator.close()
    except Exception:
        pass


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a DroneCAN dynamic node ID allocator."
    )
    parser.add_argument("--can-interface", default="can1")
    parser.add_argument("--node-id", default=127, type=int)
    parser.add_argument("--database-storage", default="")
    parser.add_argument("--spin-timeout", default=0.1, type=float)
    parser.add_argument("--retry-interval", default=1.0, type=float)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)

    try:
        dronecan, NodeMonitor, CentralizedServer = _load_dronecan()
    except ImportError as exc:
        print(
            "Failed to import dronecan. Run with the Python environment that has "
            f"dronecan installed: {exc}",
            flush=True,
        )
        return 1

    node = None
    monitor = None
    allocator = None
    active_device = ""

    while not _STOP_REQUESTED:
        if node is None:
            try:
                node, active_device = _open_node(
                    dronecan,
                    args.can_interface,
                    args.node_id,
                )
                monitor = NodeMonitor(node)
                kwargs = {}
                if args.database_storage:
                    kwargs["database_storage"] = args.database_storage
                allocator = CentralizedServer(node, monitor, **kwargs)
                print(
                    "Dynamic Node ID allocator started on "
                    f"{active_device} (local_node_id={args.node_id}).",
                    flush=True,
                )
            except Exception as exc:
                print(
                    "Dynamic Node ID allocator waiting for "
                    f"{args.can_interface}: {exc}",
                    flush=True,
                )
                time.sleep(args.retry_interval)
                continue

        try:
            node.spin(args.spin_timeout)
        except Exception as exc:
            if _is_transfer_error(dronecan, exc):
                time.sleep(0.01)
                continue

            print(
                "Dynamic Node ID allocator lost CAN connection; retrying: "
                f"{exc!r}",
                flush=True,
            )
            _close_allocator(allocator)
            node = None
            monitor = None
            allocator = None
            active_device = ""
            time.sleep(args.retry_interval)

    _close_allocator(allocator)
    del monitor
    print("Dynamic Node ID allocator stopped.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
