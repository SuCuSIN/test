import socket
import time

ROBOT_IP = "192.168.0.119"
PORT = 30002


def send_urscript(script: str):
    print(f"Connecting to {ROBOT_IP}:{PORT}...")
    with socket.create_connection((ROBOT_IP, PORT), timeout=3) as s:
        print("Connected. Sending script...")
        if not script.endswith("\n"):
            script += "\n"
        s.sendall(script.encode("utf-8"))
        time.sleep(5.0)
    print("Script sent.")


script = """
def gello_rg6_test():
  textmsg("RG6_TEST_START")

  textmsg("RG6_OPEN")
  rg_grip(100, 20)
  sleep(3.0)

  textmsg("RG6_CLOSE")
  rg_grip(20, 20)
  sleep(3.0)

  textmsg("RG6_TEST_END")
end

gello_rg6_test()
"""

send_urscript(script)
print("Done.")