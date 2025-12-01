import re

def clean_reply(text: str) -> str:
    if not text:
        return ""
    text = re.sub(r"[\U00010000-\U0010FFFF]", "", text)
    text = text.replace("😊", "").replace("🙂", "").replace("😀", "").replace("😂", "")
    text = re.sub(r"\s+", " ", text).strip()
    return text
