import argparse
import time
import platform
from time import localtime, strftime

parser = argparse.ArgumentParser()

parser.add_argument("-o", "--output", type=str, default="putty.log")
parser.add_argument("-t", "--time", type=int, default=1.01)
args = vars(parser.parse_args())
TIME_DELAY = args["time"]
filename = args["output"]

# if platform.system() == "Windows":
#     ESC = "\033["
# elif platform.system() == "Linux":
#     ESC = "\x1B["

def flush_to_file(msg: str, log_file="debug.log") -> None:
    with open(log_file, "a") as f:
        f.write(msg)
        f.flush()
def convert_to_decimal_deg(term: str, long_or_lat: str) -> str:
    if long_or_lat == "lat":
        DD = float(term[0] + term[1])
        mm_1 = term[2] + term[3]
        mm_2 = term[5] + term[6]
    elif long_or_lat == "long":
        DD = float(term[0] + term[1] + term[2])
        mm_1 = term[3] + term[4]
        mm_2 = term[6] + term[7]
    deci = float(mm_1 + "." + mm_2)
    return f"{round((DD + deci/60),3)}"

def fancy_waiting(msg: str, i: int) -> str:
    if i == 0:
        return f"{msg}."
    elif i == 1:
        return f"{msg}.."
    elif i == 2:
        return f"{msg}..."
    else:
        return f"{msg}..."
error = 0
it = -1
while True:
    if it == 2:
        it = -1
    it+=1
    current_time = strftime("%H:%M:%S", localtime())
    try:
        with open(filename, 'r', encoding="utf-8", errors="ignore") as f:
            #print(f.read())
            for line in f:
                if line.startswith("$GPGLL"):
                    parts = line.split(",")
                    last_gpgll_line = parts
        if parts[1] != "" and parts[3] != "":
            parts[1] = convert_to_decimal_deg(parts[1], "lat")
            parts[3] = convert_to_decimal_deg(parts[3], "long")
            flush_to_file(f"\x1b[93,m{current_time}, {parts[1]}, {parts[2]}, {parts[3]}, {parts[4]}\n")
        if parts[1] == "":
            parts[1] = fancy_waiting("Waiting for data", it)
        if parts[3] == "":
            parts[3] = fancy_waiting("Waiting for data", it)
        if error == 1:
            start = "\x1b[2F"
            end = "\x1b[0J"
            error = 0
        else:
            start = "\x1b[1F"
            end = "\x1b[0K"
        comma = "\x1b[97m," if parts[2] != "" else ""
        print(f"\x1b[93m{start}{current_time} \x1b[97m| \x1b[92mLong: \x1b[37m{parts[1]}{comma} \x1b[37m{parts[2]} \x1b[97m| \x1b[92mLat: \x1b[37m{parts[3]}{comma} \x1b[37m{parts[4]}{end}")
        time.sleep(TIME_DELAY)
    except Exception as e:
        print(f"\x1b[2F{current_time} | Bits lost... Looping again...\x1b[K")
        print(f"Error: {e}\x1b[K")
        error = 1
        continue

