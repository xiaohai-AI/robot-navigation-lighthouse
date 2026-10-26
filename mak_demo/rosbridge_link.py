"""极简 rosbridge 客户端：只做 agent 侧真正需要的四件事。

    publish(topic, type, msg)            发消息
    subscribe(topic, type, cb)           收消息（回调在后台线程里跑）
    call_service(service, type, args)    同步调用服务
    close()

设计取舍：
  * 不依赖 roslibpy（它偏 ROS1，action 支持不完整），只用 websocket-client 收发 JSON，
    依赖少、行为可预测、出问题看日志就懂。
  * 单条接收线程 + 按 topic 分发，避免多线程抢 socket。
  * call_service 用 id + Event 做同步等待，超时抛异常而不是静默挂死。
"""
from __future__ import annotations

import itertools
import json
import threading
import time
from typing import Any, Callable

import websocket


class RosbridgeError(RuntimeError):
    pass


class RosbridgeLink:
    def __init__(self, url: str = 'ws://127.0.0.1:9090', connect_timeout: float = 5.0):
        self.url = url
        self._ws = websocket.WebSocket()
        self._ws.settimeout(connect_timeout)
        try:
            self._ws.connect(url)
        except Exception as e:                                  # 连不上要给出可执行的建议
            raise RosbridgeError(
                f'连不上 rosbridge ({url})：{e}\n'
                f'先在另一个终端跑：ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
            ) from e
        self._ws.settimeout(None)                               # 接收线程要阻塞等

        self._subs: dict[str, Callable[[dict], None]] = {}
        self._pending: dict[str, tuple[threading.Event, list]] = {}
        self._ids = itertools.count(1)
        self._lock = threading.Lock()
        self._alive = True
        self._rx = threading.Thread(target=self._recv_loop, name='rosbridge-rx', daemon=True)
        self._rx.start()

    # ---------------- 内部 ----------------
    def _send_raw(self, obj: dict):
        data = json.dumps(obj, ensure_ascii=False)
        with self._lock:
            self._ws.send(data)

    def _recv_loop(self):
        while self._alive:
            try:
                raw = self._ws.recv()
            except Exception:
                break
            if not raw:
                continue
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            op = msg.get('op')
            if op == 'publish':
                cb = self._subs.get(msg.get('topic', ''))
                if cb is not None:
                    try:
                        cb(msg.get('msg') or {})
                    except Exception:                           # 单个回调出错不能拖垮接收线程
                        pass
            elif op == 'service_response':
                slot = self._pending.pop(msg.get('id', ''), None)
                if slot:
                    ev, box = slot
                    box.append(msg)
                    ev.set()
            elif op == 'status' and msg.get('level') == 'error':
                print(f'[rosbridge] 服务端报错: {msg.get("msg")}')

    # ---------------- 对外 ----------------
    def publish(self, topic: str, msg_type: str, msg: dict):
        self._send_raw({'op': 'publish', 'topic': topic, 'type': msg_type, 'msg': msg})

    def subscribe(self, topic: str, msg_type: str, cb: Callable[[dict], None],
                  throttle_ms: int = 0):
        self._subs[topic] = cb
        self._send_raw({'op': 'subscribe', 'topic': topic, 'type': msg_type,
                        'queue_length': 1, **({'throttle_rate': throttle_ms} if throttle_ms else {})})

    def unsubscribe(self, topic: str):
        self._subs.pop(topic, None)
        self._send_raw({'op': 'unsubscribe', 'topic': topic})

    def call_service(self, service: str, srv_type: str, args: Any = None,
                     timeout: float = 8.0) -> dict:
        sid = f'call_service:{next(self._ids)}'
        ev, box = threading.Event(), []
        self._pending[sid] = (ev, box)
        self._send_raw({'op': 'call_service', 'id': sid, 'service': service,
                        'type': srv_type, 'args': args or {}})
        if not ev.wait(timeout):
            self._pending.pop(sid, None)
            raise RosbridgeError(f'服务调用超时 {timeout}s：{service}')
        resp = box[0]
        if resp.get('result') is False:
            raise RosbridgeError(f'服务调用失败 {service}: {resp.get("values")}')
        return resp.get('values') or {}

    def topics(self) -> list[str]:
        """列出当前话题（用 rosapi，起 rosbridge 时自带）。"""
        vals = self.call_service('/rosapi/topics', 'rosapi_msgs/srv/Topics')
        return list(vals.get('topics') or [])

    def wait_for_topic(self, topic: str, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if topic in self.topics():
                return True
            time.sleep(0.5)
        return False

    def close(self):
        self._alive = False
        try:
            self._ws.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
