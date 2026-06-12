#!/usr/bin/env python3
"""
Kawasaki Ninja 7 Hybrid — Clutch Relearn Tool v2.0
Opens UDS diagnostic session, reads ECU info + DTCs, holds session for clutch relearn.

Usage:
  1. Key ON, kill switch RUN, engine OFF
  2. Run this script
  3. Wait for "Diagnostic session active" message
  4. Start engine, hold E-BOOST + START until boost gauge counts to zero
  5. Ctrl+C to stop
  6. Turn key OFF to save calibration
"""

import can
import time
import sys
import os
from datetime import datetime

# --- CONFIGURATION ---
CAN_CHANNEL = 0
CAN_BITRATE = 500000
ECU_REQUEST_ID = 0x764
ECU_RESPONSE_ID = 0x746
LOG_DIR = os.path.expanduser("~/Downloads/can-logs")

# --- UDS SERVICE HELPERS ---

def send_uds(bus, service_id, subfunction_data, description="", timeout=5.0):
    """Send a UDS request and wait for response. Returns response Message or None."""
    data = bytearray([len(subfunction_data) + 1, service_id]) + bytearray(subfunction_data)
    msg = can.Message(arbitration_id=ECU_REQUEST_ID, data=data, is_extended_id=False)
    
    try:
        bus.send(msg)
        print(f"  [SENT] 0x{ECU_REQUEST_ID:03X} | {data.hex(' ')} | {description}")
    except can.CanError as e:
        print(f"  [ERROR] Send failed: {e}")
        return None
    
    # Expected positive response: service_id + 0x40
    positive_resp = service_id + 0x40
    start = time.time()
    while time.time() - start < timeout:
        resp = bus.recv(0.5)
        if resp and resp.arbitration_id == ECU_RESPONSE_ID:
            if len(resp.data) >= 2 and resp.data[1] == positive_resp:
                return resp
            elif len(resp.data) >= 3 and resp.data[1] == 0x7F and resp.data[2] == service_id:
                nrc = resp.data[3] if len(resp.data) > 3 else 0
                nrc_names = {
                    0x11: "Service Not Supported",
                    0x12: "Sub-Function Not Supported",
                    0x13: "Incorrect Message Length",
                    0x22: "Conditions Not Correct",
                    0x31: "Request Out of Range",
                    0x33: "Security Access Denied",
                    0x72: "General Programming Failure",
                    0x78: "Request Correctly Received - Response Pending",
                }
                print(f"  [RECV] Negative Response: NRC 0x{nrc:02X} ({nrc_names.get(nrc, 'Unknown')})")
                return None
    print(f"  [RECV] Timeout — no response")
    return None


def read_did(bus, did, description="", timeout=5.0):
    """Read a Data Identifier (UDS service 0x22). Returns raw bytes or None."""
    resp = send_uds(bus, 0x22, did.to_bytes(2, 'big'), description=f"Read DID 0x{did:04X} ({description})" if description else f"Read DID 0x{did:04X}", timeout=timeout)
    if resp and len(resp.data) > 4:
        # Response format: [len] 62 DID_H DID_L [data...]
        return bytes(resp.data[4:])
    return None


def try_decode_ascii(raw_bytes):
    """Try to decode bytes as ASCII, return string or hex."""
    try:
        text = raw_bytes.decode('ascii')
        if all(32 <= ord(c) < 127 for c in text):
            return text
    except (UnicodeDecodeError, ValueError):
        pass
    return raw_bytes.hex(' ')


def read_dtcs(bus, timeout=8.0):
    """Read Diagnostic Trouble Codes (UDS service 0x19). Returns list of (code, status)."""
    # 0x19 0x02 0x08 = report DTCs by status mask (confirmed)
    # Try all DTCs first (status mask 0xFF)
    resp = send_uds(bus, 0x19, [0x02, 0xFF], description="Read DTCs (all status masks)", timeout=timeout)
    dtcs = []
    if resp and len(resp.data) >= 6:
        # Response: [len] 59 02 [availability_mask] [num_dtcs_high] [num_dtcs_low] [DTCs...]
        # DTC format: 2-byte code + 1-byte status
        payload = bytes(resp.data)
        # Skip PCI byte (0), service (1=0x59), subfunc (2=0x02), mask (3), count (4-5)
        num_dtcs = (payload[4] << 8) | payload[5] if len(payload) > 5 else 0
        print(f"  DTC count: {num_dtcs}")
        
        offset = 6
        while offset + 3 <= len(payload):
            dtc_code = (payload[offset] << 8) | payload[offset + 1]
            dtc_status = payload[offset + 2]
            dtcs.append((dtc_code, dtc_status))
            offset += 4  # some formats use 4 bytes per DTC, some 3
            # Try 3-byte format if 4-byte goes out of bounds
            if offset > len(payload) and offset - 1 + 3 <= len(payload):
                break
    
    # Also try the simpler 0x19 0x01 (count only)
    if not dtcs:
        resp = send_uds(bus, 0x19, [0x01], description="Read DTC count", timeout=timeout)
        if resp and len(resp.data) >= 5:
            num = (resp.data[3] << 8) | resp.data[4]
            print(f"  DTC count (simple): {num}")
    
    return dtcs


def format_dtc(code):
    """Format a 2-byte Kawasaki DTC as readable string."""
    # Kawasaki uses their own DTC numbering
    # Common format: first nibble = system, rest = specific code
    systems = {
        0x0: "P0", 0x1: "P1", 0x2: "P2", 0x3: "P3",
        0x4: "C0", 0x5: "C1", 0x6: "C2", 0x7: "C3",
        0x8: "B0", 0x9: "B1", 0xA: "B2", 0xB: "B3",
        0xC: "U0", 0xD: "U1", 0xE: "U2", 0xF: "U3",
    }
    prefix = systems.get((code >> 12) & 0xF, "??")
    return f"{prefix}{code & 0xFFF:04X}"


# --- ECU IDENTIFICATION ---

ECU_DIDS = {
    0xF190: "VIN",
    0xF191: "ECU Hardware Version",
    0xF192: "ECU Software Version",
    0xF193: "ECU Supplier",
    0xF194: "Manufacturing Date",
    0xF195: "ECU Serial Number",
}


def read_ecu_info(bus, log_file=None):
    """Read all standard ECU identification DIDs."""
    print("\n" + "=" * 50)
    print("ECU IDENTIFICATION")
    print("=" * 50)
    
    results = {}
    for did, desc in ECU_DIDS.items():
        raw = read_did(bus, did, description=desc)
        if raw:
            ascii_val = try_decode_ascii(raw)
            hex_val = raw.hex(' ')
            print(f"  DID 0x{did:04X} ({desc:25s}): ASCII: {ascii_val:30s} Raw: {hex_val}")
            results[did] = {"ascii": ascii_val, "hex": hex_val, "raw": raw}
        else:
            print(f"  DID 0x{did:04X} ({desc:25s}): Not available")
            results[did] = None
        
        # Small delay between DID reads
        time.sleep(0.2)
    
    if log_file:
        log_file.write(f"\n{'='*50}\nECU IDENTIFICATION — {datetime.now().isoformat()}\n{'='*50}\n")
        for did, desc in ECU_DIDS.items():
            if results.get(did):
                log_file.write(f"DID 0x{did:04X} ({desc}): ASCII={results[did]['ascii']} Raw={results[did]['hex']}\n")
            else:
                log_file.write(f"DID 0x{did:04X} ({desc}): Not available\n")
    
    return results


def read_ecu_dtcs(bus, log_file=None):
    """Read and display DTCs."""
    print("\n" + "=" * 50)
    print("DIAGNOSTIC TROUBLE CODES")
    print("=" * 50)
    
    dtcs = read_dtcs(bus)
    
    if dtcs:
        print(f"\n  Found {len(dtcs)} DTC(s):")
        for code, status in dtcs:
            status_flags = []
            if status & 0x01: status_flags.append("Test Failed")
            if status & 0x02: status_flags.append("Test Incomplete")
            if status & 0x08: status_flags.append("Confirmed")
            if status & 0x20: status_flags.append("Pending")
            status_str = ", ".join(status_flags) if status_flags else f"0x{status:02X}"
            print(f"    {format_dtc(code):8s} (raw: 0x{code:04X}) Status: {status_str}")
    else:
        print("  No DTCs found — system clean!")
    
    if log_file:
        log_file.write(f"\n{'='*50}\nDTCs — {datetime.now().isoformat()}\n{'='*50}\n")
        if dtcs:
            for code, status in dtcs:
                log_file.write(f"DTC {format_dtc(code)} (0x{code:04X}) Status: 0x{status:02X}\n")
        else:
            log_file.write("No DTCs found\n")
    
    return dtcs


# --- CAN BUS INIT ---

def initialize_bus():
    """Initializes the CAN bus based on the host Operating System."""
    tp = sys.platform
    print(f"Detected Operating System: {tp}")
    
    try:
        if tp == "linux":
            print(f"Connecting via Linux SocketCAN (interface='socketcan', channel='can{CAN_CHANNEL}')...")
            return can.interface.Bus(interface='socketcan', channel=f'can{CAN_CHANNEL}', bitrate=CAN_BITRATE)
        elif tp in ("darwin", "win32"):
            print(f"Connecting via USB abstraction layer (interface='gs_usb', channel={CAN_CHANNEL})...")
            return can.interface.Bus(interface='gs_usb', channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
        else:
            print("Unknown OS. Attempting general fallback...")
            return can.interface.Bus(channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
    except Exception as e:
        print(f"\n[ERROR] Auto-initialization failed: {e}")
        print("\nOS-Specific Verification Checklist:")
        print("  - MAC: Ensure 'brew install libusb' has been executed.")
        print("  - LINUX: Verify: 'sudo ip link set can0 up type can bitrate 500000'")
        print("  - WINDOWS: Missing libusb-1.0.dll? Download or install via pip.")
        sys.exit(1)


# --- MAIN ---

def main():
    # Create log directory
    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, f"ecu_log_{timestamp}.txt")
    
    bus = initialize_bus()
    print("Successfully connected to the CAN adapter.\n")
    
    # Open log file
    with open(log_path, 'w') as log_file:
        log_file.write(f"Kawasaki Ninja 7 Hybrid — ECU Log\n")
        log_file.write(f"Started: {datetime.now().isoformat()}\n")
        log_file.write(f"CAN: 500kbps, Request ID: 0x{ECU_REQUEST_ID:03X}, Response ID: 0x{ECU_RESPONSE_ID:03X}\n")
        
        # --- STEP 1: Open diagnostic session ---
        print("=" * 50)
        print("OPENING DIAGNOSTIC SESSION")
        print("=" * 50)
        
        init_msg = can.Message(arbitration_id=ECU_REQUEST_ID, data=[0x02, 0x10, 0x80], is_extended_id=False)
        try:
            bus.send(init_msg)
            print(f"[SENT] ID: 0x{ECU_REQUEST_ID:03X} | Data: {init_msg.data.hex(' ')}")
            log_file.write(f"[SENT] 0x{ECU_REQUEST_ID:03X} {init_msg.data.hex(' ')} — Diagnostic Session Request\n")
        except can.CanError as e:
            print(f"[ERROR] Failed to send: {e}")
            bus.shutdown()
            sys.exit(1)
        
        print("Watching for response ID: 0x{:03X}...".format(ECU_RESPONSE_ID))
        response_received = False
        timeout = 10.0
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = bus.recv(1.0)
            if msg and msg.arbitration_id == ECU_RESPONSE_ID:
                if len(msg.data) >= 3 and msg.data[0:3] == bytearray([0x02, 0x50, 0x80]):
                    print(f"[RECV] Valid response captured! Data: {msg.data.hex(' ')}\n")
                    log_file.write(f"[RECV] 0x{ECU_RESPONSE_ID:03X} {msg.data.hex(' ')} — Session Accepted\n")
                    response_received = True
                    break
        
        if not response_received:
            print("[ERROR] Timeout — no valid response. Check key switch, kill switch, wiring.")
            log_file.write("[ERROR] Session handshake failed — timeout\n")
            bus.shutdown()
            sys.exit(1)
        
        # --- STEP 2: Read ECU identification ---
        ecu_info = read_ecu_info(bus, log_file)
        
        # --- STEP 3: Read DTCs ---
        dtcs = read_ecu_dtcs(bus, log_file)
        
        # --- STEP 4: Hold session for clutch relearn ---
        print("\n" + "=" * 50)
        print("DIAGNOSTIC SESSION ACTIVE")
        print("=" * 50)
        print("  1. Start the engine")
        print("  2. Hold E-BOOST + START simultaneously")
        print("  3. Wait for boost gauge to count down to zero")
        print("  4. Ctrl+C to stop this script")
        print("  5. Turn key OFF to save calibration")
        print("=" * 50)
        print()
        
        tester_present_msg = can.Message(arbitration_id=ECU_REQUEST_ID, data=[0x01, 0x3E], is_extended_id=False)
        last_send_time = 0
        
        try:
            while True:
                current_time = time.time()
                if current_time - last_send_time >= 2.0:
                    try:
                        bus.send(tester_present_msg)
                        print(f"[KEEP-ALIVE] Tester Present | {time.strftime('%H:%M:%S')}")
                        log_file.write(f"{time.strftime('%H:%M:%S')} [SENT] Tester Present\n")
                        last_send_time = current_time
                    except can.CanOperationError as e:
                        print(f"[CAN ERROR] {e}")
                        time.sleep(1)
                
                incoming = bus.recv(0.1)
                if incoming:
                    # Log all traffic but only display important frames
                    log_file.write(f"{time.strftime('%H:%M:%S')} [BUS] 0x{incoming.arbitration_id:03X} {incoming.data.hex(' ')}\n")
                    # Only print diagnostic responses, not regular bus traffic
                    if incoming.arbitration_id == ECU_RESPONSE_ID:
                        print(f"[ECU] ID: 0x{incoming.arbitration_id:03X} | Data: {incoming.data.hex(' ')}")
        
        except KeyboardInterrupt:
            print("\n[INFO] Script stopped by user.")
            log_file.write(f"\n[INFO] Script stopped by user at {datetime.now().isoformat()}\n")
        finally:
            log_file.write(f"\nSession ended: {datetime.now().isoformat()}\n")
            bus.shutdown()
            print("Shutdown complete.")
    
    print(f"\nLog saved to: {log_path}")


if __name__ == "__main__":
    main()