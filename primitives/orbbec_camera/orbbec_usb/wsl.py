"""Launch the Windows SDK even if WSLInterop's binfmt handler is missing."""
import errno
import platform
import shutil
from pathlib import Path


def run(process_api, command, **kwargs):
    try:
        return process_api(command, **kwargs)
    except OSError as error:
        if (error.errno != errno.ENOEXEC or "microsoft" not in platform.release().lower()
                or Path(command[0]).suffix.lower() != ".exe"):
            raise
        executable = shutil.which(command[0])
        if executable is None:
            raise
        # Reproduce binfmt's preserve-argv form: interpreter, binary path, original argv.
        return process_api(["/init", executable, executable, *command[1:]], **kwargs)
