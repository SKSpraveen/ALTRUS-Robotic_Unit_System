import os
import shutil
from pathlib import Path

def ensure_ffmpeg_available(ffmpeg_exe: str | None) -> bool:
    """
    If ffmpeg_exe is provided, add it to PATH.
    Otherwise confirm ffmpeg is available in PATH.
    """
    if ffmpeg_exe:
        p = Path(ffmpeg_exe)
        if p.is_file():
            os.environ["PATH"] = str(p.parent) + os.pathsep + os.environ.get("PATH", "")
            return True
        print(f"[FFMPEG] Provided path not found: {ffmpeg_exe}")
        return False

    found = shutil.which("ffmpeg")
    if found:
        return True

    print("[FFMPEG] ffmpeg not found in PATH.")
    return False
