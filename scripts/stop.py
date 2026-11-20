#!/usr/bin/env python3
"""Call the running provider's stop tool through Executor, without an LLM."""
import json

from check_runtime import call_tools


def main():
    print(json.dumps(call_tools([("stop", {})]), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
