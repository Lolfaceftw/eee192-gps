import argparse
import time
from time import localtime, strftime

parser = argparse.ArgumentParser()

parser.add_argument("-o", "--output", type=str, default="putty.log")
parser.add_argument("-t", "--time", type=int, default=1.01)
args = vars(parser.parse_args())
TIME_DELAY = args["time"]
filename = args["output"]


UP = "\x1B[3A"
CLR = "\x1B[0K"

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

while True:
    current_time = strftime("%H:%M:%S", localtime())
    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith("$GPGLL"):
                    parts = line.split(",")
                    last_gpgll_line = parts
        if parts[1] != "" and parts[3] != "":
            parts[1] = convert_to_decimal_deg(parts[1], "lat")
            parts[3] = convert_to_decimal_deg(parts[3], "long")
            flush_to_file(f"{current_time}, {parts[1]}, {parts[2]}, {parts[3]}, {parts[4]}\n")
        if parts[1] == "":
            parts[1] = "Waiting for data..."
        if parts[3] == "":
            parts[3] = "Waiting for data..."
        print(f"\x1b[F{current_time} | Long: {parts[1]}, {parts[2]} | Lat: {parts[3]}, {parts[4]}\x1b[K")
        time.sleep(TIME_DELAY)
    except:
        print(f"\x1b[F{current_time} | Bits lost... Looping again...\x1b[K")
        continue

