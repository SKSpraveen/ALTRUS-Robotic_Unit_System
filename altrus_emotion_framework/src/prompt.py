import numpy as np

def label_to_text(label: int, labels: tuple[str, ...]) -> str:
    return labels[label] if 0 <= label < len(labels) else "unknown"

def build_prompt(
    user_text: str,
    fused_label: int,
    fused_probs: np.ndarray,
    labels: tuple[str, ...],
) -> str:
    emotion = label_to_text(fused_label, labels)
    happy_p = float(fused_probs[0])
    sad_p = float(fused_probs[1])
    confidence = max(happy_p, sad_p)

    if not user_text:
        user_text = "(Speech transcription unavailable)"

    prompt = f"""
You are an empathetic assistant in a robot laptop.
You will respond to the user based on:
1) What the user said (text)
2) The user's emotion detected from face+voice

Rules:
- Be kind, supportive, and natural.
- NO emojis, NO emoticons, NO special symbols.
- Do not mention models, probabilities, sensors, face, voice, or fusion.
- If emotion is sad: comfort and encourage.
- If emotion is happy: celebrate with them.

Detected emotion: {emotion}
Confidence: {confidence:.2f} (happy={happy_p:.2f}, sad={sad_p:.2f})
User said: {user_text}

Now reply:
""".strip()

    return prompt
