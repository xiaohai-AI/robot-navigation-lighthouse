"""对照实验：选项集大小对 VLM 准确率/延迟的影响（用已保存的帧，不用开车）。

问题背景：原有 10 个固定选项只覆盖 5/33 类别，期望类别不在选项里时，
模型必然被判"不符"——这是假告警，不是模型缺陷。

跑法：.venv/bin/python dev/probe_options.py
"""
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import vision                                    # noqa: E402
from semantic_map import SemanticMap             # noqa: E402

FRAME = HERE / 'logs' / 'look_231533.png'        # 真机验收那一帧（冰箱）
SMALL = ('冰箱', '橱柜', '门', '墙', '床', '沙发', '桌椅', '垃圾桶', '书架', '看不出')


def main():
    if not FRAME.exists():
        print(f'❌ 找不到 {FRAME}，先跑一次 dev/test_look_at.py')
        return 1
    png = FRAME.read_bytes()
    sm = SemanticMap()
    full = sorted({sm.class_zh(o['class']) for o in sm.objects}) + ['看不出']
    print(f'帧 {FRAME.name}  {len(png) / 1024:.1f} KB')
    print(f'小选项集 {len(SMALL)} 个；全选项集 {len(full)} 个（覆盖 33/33）')
    print('=' * 72)
    for name, opts in (('小选项集(10)', SMALL), ('全选项集(34)', full)):
        prompt = vision.build_prompt(opts)
        t0 = time.time()
        text, el = vision.ask_vlm(png, prompt)
        ans, why = vision.parse_answer(text)
        print(f'\n【{name}】{el:.1f}s  → 答案「{ans}」 依据「{why}」')
        print(f'   判对：{"✅" if ans == "冰箱" else "❌"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
