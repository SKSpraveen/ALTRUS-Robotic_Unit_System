import numpy as np

def fuse_probs(face_probs, voice_probs, w_face: float = 0.6, w_voice: float = 0.4):
    fused = w_face * np.array(face_probs) + w_voice * np.array(voice_probs)
    label = int(np.argmax(fused))
    return fused, label

