import os
import re
import matplotlib.pyplot as plt

# Define the folder path and screw leads
folder_path = r"C:\\Users\\CMcginty\\OneDrive - AMETEK Inc\\Documents\\Torque Gauge\\Serial Data"
screw_leads = {
    "TorqueData_TestScrew1": 0.25,
    "TorqueData_TestScrew2": 0.25,
    "TorqueData_TestScrew3": 0.25,
    "TorqueData_TestScrew4": 0.6,
    "TorqueData_NoLoad": 0.0
}

# Function to parse a single file and return sessions
def parse_file(file_path, lead):
    sessions = []
    with open(file_path, 'r') as f:
        content = f.read()

    # Split into sessions
    raw_sessions = content.split("=== BEGIN SESSION ===")
    position = 0
    time_offset = 0

    for raw in raw_sessions[1:]:
        header, *data_and_footer = raw.strip().split("\n")
        spins = int(re.search(r"Spins: (\d+)", raw).group(1))
        direction = re.search(r"Direction: (\w+)", raw).group(1)
        start_time = int(re.search(r"Start Time: (\d+)", raw).group(1))
        end_time = int(re.search(r"End Time: (\d+)", raw).group(1))

        direction_sign = 1 if direction == "Forwards" else -1
        torque_values = [int(line) for line in raw.split("End Time:")[0].splitlines() if line.strip().isdigit()]

        elapsed_time = [time_offset + i for i in range(len(torque_values))]
        position_values = [position + direction_sign * lead * spins * (i / len(torque_values)) for i in range(len(torque_values))]

        sessions.append({
            "torque": torque_values,
            "time": elapsed_time,
            "position": position_values
        })

        position += direction_sign * lead * spins
        time_offset += len(torque_values)

    return sessions

# Collect all sessions by screw type
all_data = {}
for file_name in screw_leads:
    full_path = os.path.join(folder_path, file_name + ".txt")
    if os.path.exists(full_path):
        all_data[file_name] = parse_file(full_path, screw_leads[file_name])

# Plotting
for screw, sessions in all_data.items():
    plt.figure(figsize=(10, 5))
    for session in sessions:
        plt.plot(session["time"], session["torque"], label="Session")
    plt.title(f"Torque vs Time - {screw}")
    plt.xlabel("Elapsed Time")
    plt.ylabel("Torque")
    plt.grid(True)
    plt.show()

    plt.figure(figsize=(10, 5))
    for session in sessions:
        plt.plot(session["position"], session["torque"], label="Session")
    plt.title(f"Torque vs Position - {screw}")
    plt.xlabel("Position (inches)")
    plt.ylabel("Torque")
    plt.grid(True)
    plt.show()