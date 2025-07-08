# log_utils.py

from datetime import datetime
import os

def log_print(*args, log_file_path='log_output.txt', sep=' ', end='\n', flush=False):
    """
    A custom printf-like function that prints messages to the screen and writes them to a log file with timestamp.

    Args:
        *args: Variable-length argument list to print.
        log_file_path (str): Path to the log file. Default is 'log_output.txt'.
        sep (str): Separator between arguments, same as print(). Default is space.
        end (str): End character after the message, same as print(). Default is newline.
        flush (bool): Whether to forcibly flush the print buffer. Default is False.
    """
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    message = sep.join(str(arg) for arg in args)
    final_output = f"[{timestamp}] {message}{end}"

    # Ensure log directory exists
    os.makedirs(os.path.dirname(log_file_path), exist_ok=True)

    # Print to console
    print(final_output, end='', flush=flush)

    # Append to log file
    with open(log_file_path, 'a', encoding='utf-8') as f:
        f.write(final_output)
