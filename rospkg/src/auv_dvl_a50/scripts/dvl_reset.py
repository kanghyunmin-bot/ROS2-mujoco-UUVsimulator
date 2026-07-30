#!/usr/bin/env python3
import socket, json, argparse, sys, time

def reset_dead_reckoning(ip: str, port: int, timeout: float = 2.0, delay: float = 0.06) -> int:
    """
    DVL에 reset_dead_reckoning 1회 전송 후 응답 확인.
    success면 0, 실패/예외면 1 반환(프로세스 종료 코드 용).
    """
    payload = json.dumps({"command": "reset_dead_reckoning"}) + "\n"
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((ip, port))
        sock.sendall(payload.encode("utf-8"))

        # 한 줄(JSON) 응답 수신
        buf, start = [], time.time()
        while True:
            if time.time() - start > timeout:
                raise TimeoutError("No response within timeout")
            ch = sock.recv(1)
            if not ch:
                raise ConnectionError("Socket closed by peer")
            buf.append(ch.decode("utf-8", errors="ignore"))
            if buf[-1] == "\n":
                break

        line = "".join(buf).strip()
        resp = json.loads(line) if line else {}
        ok = bool(resp.get("success", False))
        if ok:
            # 문서상 약 50ms 후부터 zeroing — 살짝 기다렸다가 종료
            time.sleep(delay)
            print("[OK] DVL dead reckoning reset successful.")
            return 0
        else:
            emsg = resp.get("error_message", "Unknown error")
            print(f"[FAIL] DVL reset failed: {emsg}")
            return 1

    except Exception as e:
        print(f"[ERROR] {e}")
        return 1
    finally:
        try:
            if sock:
                sock.close()
        except Exception:
            pass

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Reset Water Linked DVL dead reckoning once.")
    ap.add_argument("--ip",   default="10.42.0.186", help="DVL IP (default: 10.42.0.186)")
    ap.add_argument("--port", type=int, default=16171, help="DVL TCP JSON port (default: 16171)")
    ap.add_argument("--timeout", type=float, default=2.0, help="socket timeout seconds (default: 2.0)")
    ap.add_argument("--delay",   type=float, default=0.06, help="post-success delay seconds (default: 0.06)")
    args = ap.parse_args()
    sys.exit(reset_dead_reckoning(args.ip, args.port, args.timeout, args.delay))
