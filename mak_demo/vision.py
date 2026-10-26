"""视觉模块：抓一帧 + 问 VLM 一个**封闭问题**。

两条被实测逼出来的规则（别改成开放式）：
  ① 相机必须离物体表面 ~1.40 m、离地 0.70 m，门缝和把手才在画面里；
     0.117 m 高时只框到面板下部 0.57 m 高的一带，VLM 会把冰箱判成"房门"。
  ② 提问必须是封闭选项 + 固定输出格式。开放式"描述画面里有哪些物体"
     会让 8B 模型编造场景（实测把厨房地砖说成床单、把冰箱说成房门），
     而且耗时从 2.5 s 涨到 20~41 s。

为什么只做"确认"而不是"理解"：
    VLM 单次 2.5~26 s，放不进控制环；但作为"到点后的一步确认"完全够用，
    而且它回答的是地图不知道的事（物体还在不在、门开没开、通道有没有被挡）。
"""
from __future__ import annotations

import base64
import json
import re
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOGS = HERE / 'logs'
OLLAMA = 'http://127.0.0.1:11434/api/chat'
VLM_MODEL = 'qwen3-vl:8b'

# 封闭选项：与 lexicon 的中文正名保持一致，便于和语义地图自动比对。
# 注意：这只是**小集合**，只覆盖 5/33 个类别。若被确认的类别不在其中
# （椅子/餐桌/马桶/衣柜/绿植…共 28 种），模型只能从错的候选里挑 → 必然假告警。
# 因此 verify_object() 默认用全类别集（见 build_prompt / full_options）。
OPTIONS = ('冰箱', '橱柜', '门', '墙', '床', '沙发', '桌椅', '垃圾桶', '书架', '看不出')


def full_options(expected_zh: str = '') -> tuple:
    """语义地图里的全部类别 + 看不出，并确保 expected 一定在内。"""
    opts: list = []
    try:
        from semantic_map import SemanticMap
        sm = SemanticMap()
        opts = sorted({sm.class_zh(o['class']) for o in sm.objects})
    except Exception:                                  # 没有地图时退回小集合
        opts = list(OPTIONS)
    if expected_zh and expected_zh not in opts:
        opts.append(expected_zh)
    if '看不出' not in opts:
        opts.append('看不出')
    return tuple(opts)


def build_prompt(options=None) -> str:
    opts = tuple(options) if options else OPTIONS
    return (
        '这是家用机器人的前置相机画面（离地 0.6 m，正对前方物体）。'
        '只回答一个问题：画面中央的主体物体是下列哪一个？\n'
        f'选项：{" / ".join(opts)}\n'
        '只输出一行，格式严格为：答案=<选项>；依据=<不超过15字，指出你看到的结构特征>\n'
        '如果画面里只有墙面或地面，答案填「看不出」。不要描述画面其他部分。'
    )


PROMPT = build_prompt()


def to_bytes(data) -> bytes:
    if isinstance(data, bytes):
        return data
    if isinstance(data, list):
        return bytes(b & 0xFF for b in data)
    if isinstance(data, dict):
        return to_bytes(data.get('data', []))
    return base64.b64decode(data)


def capture_frame(link=None, timeout=8.0):
    """抓一帧，返回 (png_bytes, path)。png 比 raw base64 小约 5 倍（58 KB vs 300 KB）。"""
    from PIL import Image

    own = link is None
    if own:
        from rosbridge_link import RosbridgeLink
        link = RosbridgeLink()
    got = []

    def on_img(m):
        got.append(m)

    link.subscribe('/camera/image_raw', 'sensor_msgs/Image', on_img)
    t0 = time.time()
    while time.time() - t0 < timeout and not got:
        time.sleep(0.05)
    if own:
        link.close()
    if not got:
        raise RuntimeError('抓帧超时：仿真/ros_gz_bridge 在跑吗？')

    m = got[-1]
    w, h = int(m['width']), int(m['height'])
    img = Image.frombytes('RGB', (w, h), to_bytes(m['data'])[:w * h * 3])
    LOGS.mkdir(exist_ok=True)
    p = LOGS / f"look_{time.strftime('%H%M%S')}.png"
    img.save(p)
    return p.read_bytes(), p


def _post(png: bytes, prompt: str, model: str, num_predict: int) -> dict:
    body = json.dumps({
        'model': model,
        'stream': False,
        'options': {'temperature': 0, 'num_predict': num_predict},
        'messages': [{'role': 'user', 'content': prompt,
                      'images': [base64.b64encode(png).decode()]}],
    }).encode()
    req = urllib.request.Request(OLLAMA, data=body,
                                 headers={'Content-Type': 'application/json'})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=240) as r:
        d = json.loads(r.read())
    msg = d.get('message', {}) or {}
    return {'content': msg.get('content') or '',
            'thinking': msg.get('thinking') or '',
            'done_reason': d.get('done_reason'),
            'eval_count': d.get('eval_count'),
            'elapsed': time.time() - t0}


def ask_vlm_detailed(png: bytes, prompt: str = PROMPT, model: str = VLM_MODEL,
                     num_predict: int = 600) -> dict:
    """问 VLM，返回带诊断信息的 dict（content/thinking/done_reason/elapsed/…）。

    为什么默认 600 而不是 80：qwen3-vl 是 **thinking 模型**，token 预算会先被思考吃掉。
    实测同一帧、同一个问题：

        num_predict=80   2.1 s  done=length  content=''   thinking=135 字   ← 空回答
        num_predict=200  3.8 s  done=length  content=''   thinking=322 字   ← 仍然空
        num_predict=600  6.1 s  done=stop    content=14 字 thinking=500 字 ← 「畅通」，正确

    也实测过两条"关思考"的老办法，**在这版 ollama 上都不生效**：
      · `think: False` —— 思考内容照样输出
      · prompt 加 `/no_think` —— 仍然 done=length、content 空

    所以策略是：给足预算 + 截断检测 + 翻倍重试 + 从思考末尾兜底捞答案。
    """
    r = _post(png, prompt, model, num_predict)
    r['truncated'] = (r['done_reason'] == 'length' and not r['content'].strip())
    if r['truncated']:
        r2 = _post(png, prompt, model, num_predict * 3)
        r2['retried'] = True
        r2['truncated'] = (r2['done_reason'] == 'length' and not r2['content'].strip())
        r = r2
    if not r['content'].strip() and r['thinking']:
        m = re.search(r'答案\s*[=＝]\s*([^\s；;，,]+)', r['thinking'][-300:])
        if m:
            r['content'] = f'答案={m.group(1)}；依据=（模型被截断，从思考末尾恢复）'
            r['salvaged'] = True
            r['truncated'] = False
    return r


def ask_vlm(png: bytes, prompt: str = PROMPT, model: str = VLM_MODEL,
            num_predict: int = 600):
    """问 VLM，返回 (回答文本, 耗时秒)。诊断细节用 ask_vlm_detailed()。"""
    r = ask_vlm_detailed(png, prompt, model, num_predict)
    return r['content'], r['elapsed']


def parse_answer(text: str) -> tuple:
    """从 '答案=冰箱；依据=竖把手+门缝' 里取出 (答案, 依据)。取不到就返回 ('', 原文)。"""
    ans, why = '', text.strip()
    for seg in text.replace('\n', '；').split('；'):
        seg = seg.strip()
        if seg.startswith('答案'):
            ans = seg.split('=', 1)[-1].strip(' ：:')
        elif seg.startswith('依据'):
            why = seg.split('=', 1)[-1].strip(' ：:')
    return ans, why


def verify_object(expected_zh: str, model: str = VLM_MODEL, options=None):
    """到点后的视觉确认：抓帧 → 封闭提问 → 与语义地图的预期比对。

    默认用**全类别选项集**（语义地图 33 类 + 看不出），而不是 10 个小集合：
    小集合只覆盖 5/33，期望类别不在其中时模型只能从错的候选里挑，
    会被判成"视觉与地图不符"——那是假告警，不是模型的问题。

    返回 dict：answer / evidence / match(bool|None) / latency / frame
    match=None 表示 VLM 说"看不出"，既不算对也不算错 —— 由调用方决定怎么处理。
    """
    opts = tuple(options) if options else full_options(expected_zh)
    png, path = capture_frame()
    r = ask_vlm_detailed(png, build_prompt(opts), model=model)
    ans, why = parse_answer(r['content'])
    if ans == '看不出' or not ans:
        match = None
    else:
        match = (ans == expected_zh)
    return {'answer': ans, 'evidence': why, 'match': match,
            'latency': r['elapsed'], 'frame': str(path), 'raw': r['content'],
            'options': len(opts), 'truncated': r.get('truncated', False)}


# ---------------- 可通行性问答（相机真正擅长的活） ----------------
# 为什么换问题：物体身份识别在这台机器的取景几何下本来就吃亏（见 docs/phase2B_vision.md
# §2/§3），而"正前方能不能过"恰好落在相机的强项上 —— 贴地的线缆、门槛、地毯边、
# 透明玻璃这些 2D 雷达在 0.132 m 平面扫不到的东西，画面里一眼就能看见。
PASS_OPTIONS = ('畅通', '被挡住', '看不清')


def build_pass_prompt(options=PASS_OPTIONS) -> str:
    return (
        '这是家用机器人的前置相机画面（离地 0.6 m，正对前进方向）。\n'
        '只回答一个问题：画面正中的前方通道，机器人能不能直接穿过去？\n'
        f'选项：{" / ".join(options)}\n'
        '只输出一行：答案=<选项>；依据=<不超过15字，指出挡路的东西或畅通的依据>\n'
        '注意：地面上的线缆、门槛、地毯边、透明玻璃都算阻挡；只是"地面有花纹"不算。'
    )


def assess_passability(model: str = VLM_MODEL):
    """抓一帧 → 问"正前方能否通过" → 返回 {answer, evidence, latency, frame, truncated}。"""
    png, path = capture_frame()
    r = ask_vlm_detailed(png, build_pass_prompt(), model=model)
    ans, why = parse_answer(r['content'])
    if not ans:
        # 别把"模型没吐字"伪装成"看不清" —— 标出来，调用方要如实说
        ans, why = '看不清', ('模型输出被思考截断' if r.get('truncated')
                              else '模型没有给出可解析的答案')
    return {'answer': ans, 'evidence': why, 'latency': r['elapsed'],
            'frame': str(path), 'raw': r['content'],
            'truncated': r.get('truncated', False)}
