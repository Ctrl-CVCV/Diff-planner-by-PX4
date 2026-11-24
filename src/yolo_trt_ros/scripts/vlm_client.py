"""Ask the local Qwen2.5-VL server what is inside one colored frame."""
import base64
import json
import time
import urllib.error
import urllib.request

import cv2

SIGN_PROMPT = (
    '判断方框内的标志属于哪一类，只输出一个标签，不要解释。'
    '危险标志：易燃、易爆、有毒有害、腐蚀等危险警示标志。'
    '非危险标志：行人、注意等普通警告标志，或其它非危险标志。'
    '只能从这两项中选一项：危险标志、非危险标志。'
)
SIGN_GRAMMAR = 'root ::= "危险标志" | "非危险标志"'
PORTRAIT_PROMPT = (
    '观察方框内的画面，判断主体是不是人物头像，只输出一个标签，不要解释。'
    '人物头像：真实的人头或人脸，可以是照片。'
    '不是人物头像：风景、绘画、标志、物品，或没有清楚的人头。'
    '只能从这两项中选一项：人物头像、不是人物头像。'
)
PORTRAIT_GRAMMAR = 'root ::= "人物头像" | "不是人物头像"'


class VisionError(RuntimeError):
    pass


def classify_frame(image, border_color, endpoint, model='qwen2.5-vl-7b',
                   timeout=60.0, max_image_side=640):
    """Return (text, elapsed_ms) for a red sign frame or a blue portrait frame."""
    if border_color == 'red':
        prompt, grammar = SIGN_PROMPT, SIGN_GRAMMAR
    elif border_color == 'blue':
        prompt, grammar = PORTRAIT_PROMPT, PORTRAIT_GRAMMAR
    else:
        raise ValueError('border_color must be red or blue')
    data_url = _jpeg_data_url(image, max_image_side)
    body = {
        'model': model,
        'messages': [{'role': 'user', 'content': [
            {'type': 'text', 'text': prompt},
            {'type': 'image_url', 'image_url': {'url': data_url}},
        ]}],
        'max_tokens': 16,
        'temperature': 0.0,
        'grammar': grammar,
        'cache_prompt': False,
        'stream': False,
    }
    request = urllib.request.Request(
        endpoint, data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
        headers={'Content-Type': 'application/json'}, method='POST')
    started = time.perf_counter()
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            payload = json.loads(response.read().decode('utf-8'))
        text = payload['choices'][0]['message']['content']
        if isinstance(text, list):
            text = ''.join(part.get('text', '') for part in text)
        text = str(text).strip()
        if not text:
            raise ValueError('empty model text')
    except (urllib.error.URLError, TimeoutError, OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
        raise VisionError(str(exc)) from exc
    return text, (time.perf_counter() - started) * 1000.0


def _jpeg_data_url(image, max_image_side):
    height, width = image.shape[:2]
    scale = min(1.0, float(max_image_side) / max(width, height))
    if scale < 1.0:
        image = cv2.resize(image, (max(1, round(width * scale)),
                                   max(1, round(height * scale))),
                           interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise VisionError('JPEG encoding failed')
    payload = base64.b64encode(encoded.tobytes()).decode('ascii')
    return 'data:image/jpeg;base64,' + payload
