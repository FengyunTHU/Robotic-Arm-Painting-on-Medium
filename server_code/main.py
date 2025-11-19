from Points import process_svg as ps_svg
from Communication import ping_port as ping
from Communication import send_json as send_J

if __name__ == "__main__":
    # ping("COM9", 115200, "ALPHABET\n", 5.0)
    send_J("COM9", 115200, "./JSON/1_points.json", retries=3, resp_timeout=10.0, inter_chunk_delay=0.01)