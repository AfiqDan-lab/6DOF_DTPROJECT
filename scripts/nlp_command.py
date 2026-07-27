#!/usr/bin/env python3
"""
nlp_command.py  -  Phase 4a: turn plain English into a target position.

"move to the left bin"  ->  {"x": 0.30, "y": 0.30, "z": 0.20}

It tries a local language model through Ollama (your proposal's Gemma/Qwen).
If Ollama isn't running, it falls back to a simple built-in parser, so you can
test the pipeline immediately and add the real LLM whenever you like.

    python scripts/nlp_command.py                    # runs demo commands, then a prompt
    python scripts/nlp_command.py "reach up high"    # interpret one command

To enable the real language model (optional, recommended for the capstone):
    1) install Ollama from https://ollama.com   (has a Windows installer)
    2) pull a small model:   ollama pull qwen2.5     (or: ollama pull gemma2)
    3) leave Ollama running; this script will use it automatically.
"""
import json
import re
import sys
import urllib.request

# ---- the arm's "world": named spots (metres) and reachable bounds ----
NAMED_LOCATIONS = {
    "home":      (0.25, 0.00, 0.55),
    "left bin":  (0.30, 0.30, 0.20),
    "right bin": (0.30, -0.30, 0.20),
    "center":    (0.38, 0.00, 0.35),
    "drop zone": (0.20, 0.30, 0.30),
}
WORKSPACE = {"x": (0.12, 0.48), "y": (-0.38, 0.38), "z": (0.12, 0.85)}

# ---- local LLM (optional) ----
OLLAMA_URL = "http://localhost:11434"
MODEL = "qwen2.5"   # or "gemma2", "llama3.2", etc. -- must be `ollama pull`ed first


def clamp(x, y, z):
    return (min(max(x, WORKSPACE["x"][0]), WORKSPACE["x"][1]),
            min(max(y, WORKSPACE["y"][0]), WORKSPACE["y"][1]),
            min(max(z, WORKSPACE["z"][0]), WORKSPACE["z"][1]))


def resolve_target(data):
    """Accept {'x','y','z'} or {'location': name} from either interpreter."""
    if data is None:
        return None
    if "location" in data and data["location"] in NAMED_LOCATIONS:
        return NAMED_LOCATIONS[data["location"]]
    if all(k in data for k in ("x", "y", "z")):
        return (float(data["x"]), float(data["y"]), float(data["z"]))
    return None


# --------------------------------------------------------------------------
# option 1: the local language model (Ollama)
# --------------------------------------------------------------------------
def build_prompt(text):
    locs = "\n".join(f'  - "{k}": x={v[0]}, y={v[1]}, z={v[2]}'
                     for k, v in NAMED_LOCATIONS.items())
    return (
        "You convert a spoken command into a target position for a 6-DOF robot arm.\n\n"
        f"Named locations (metres):\n{locs}\n\n"
        f"Workspace limits: x in {WORKSPACE['x']}, y in {WORKSPACE['y']}, "
        f"z in {WORKSPACE['z']}.\n\n"
        "AXES - read carefully:\n"
        "  LEFT  = y POSITIVE  (for example y = +0.30)\n"
        "  RIGHT = y NEGATIVE  (for example y = -0.30)\n"
        "  UP = larger z, DOWN = smaller z\n"
        "  FORWARD (away from the base) = larger x, BACK = smaller x\n"
        "  Change only the axes the command mentions; for the rest use the "
        "'center' values.\n\n"
        'Reply with ONLY one JSON object: a short "reason" first, then x, y, z.\n'
        "Examples:\n"
        'Command: "go to the left bin"\n'
        '{"reason":"left bin is a named location","x":0.30,"y":0.30,"z":0.20}\n'
        'Command: "move to the right"\n'
        '{"reason":"right means negative y","x":0.38,"y":-0.30,"z":0.35}\n'
        'Command: "reach up"\n'
        '{"reason":"up means larger z","x":0.38,"y":0.00,"z":0.60}\n'
        'Command: "move down and to the left"\n'
        '{"reason":"left is +y, down is smaller z","x":0.38,"y":0.30,"z":0.20}\n'
        'Command: "go to x 0.2 y -0.1 z 0.4"\n'
        '{"reason":"explicit coordinates","x":0.20,"y":-0.10,"z":0.40}\n\n'
        f'Command: "{text}"'
    )


def ollama_interpret(text):
    """Return a target dict from the LLM, or None if Ollama is unavailable."""
    body = json.dumps({"model": MODEL, "prompt": build_prompt(text),
                       "stream": False, "format": "json"}).encode()
    req = urllib.request.Request(OLLAMA_URL + "/api/generate", data=body,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            response = json.loads(r.read())
        return json.loads(response["response"])   # format=json -> valid JSON string
    except Exception:
        return None


# --------------------------------------------------------------------------
# option 2: built-in fallback parser (no LLM needed)
# --------------------------------------------------------------------------
def rule_interpret(text):
    t = text.lower()

    # named location?
    for name, xyz in NAMED_LOCATIONS.items():
        if name in t:
            return {"x": xyz[0], "y": xyz[1], "z": xyz[2]}

    # explicit coordinates like "x 0.3 y 0.1 z 0.5" or "x=0.3, y=0.1, z=0.5"?
    nums = {ax: re.search(rf"{ax}\s*=?\s*(-?\d+\.?\d*)", t) for ax in "xyz"}
    if all(nums.values()):
        return {ax: float(m.group(1)) for ax, m in nums.items()}

    # relative words, applied from "center"
    x, y, z = NAMED_LOCATIONS["center"]
    step = 0.15
    if "left" in t:    y += step
    if "right" in t:   y -= step
    if "up" in t or "high" in t:    z += step
    if "down" in t or "low" in t:   z -= step
    if "forward" in t or "far" in t: x += step
    if "back" in t or "close" in t:  x -= step
    return {"x": x, "y": y, "z": z}


# --------------------------------------------------------------------------
# combined entry point
# --------------------------------------------------------------------------
def interpret(text):
    """Return {'x','y','z','method'} for a command (LLM if available, else rules)."""
    data = ollama_interpret(text)
    method = f"LLM ({MODEL})"
    if resolve_target(data) is None:
        data = rule_interpret(text)
        method = "built-in parser"
    x, y, z = clamp(*resolve_target(data))
    return {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3), "method": method}


def show(text):
    r = interpret(text)
    print(f'  "{text}"'.ljust(38)
          + f"-> ({r['x']:+.2f}, {r['y']:+.2f}, {r['z']:+.2f})   [{r['method']}]")


def main():
    if len(sys.argv) > 1:
        show(" ".join(sys.argv[1:]))
        return

    print("\nDemo commands:")
    for cmd in ["move to the left bin", "go to the right bin", "return home",
                "swing up as high as you can", "shift left and lower it",
                "go to x 0.4 y -0.2 z 0.3"]:
        show(cmd)

    print("\nType a command (blank or Ctrl+C to quit):")
    try:
        while True:
            line = input("> ").strip()
            if not line:
                break
            show(line)
    except (EOFError, KeyboardInterrupt):
        pass
    print("bye")


if __name__ == "__main__":
    main()
